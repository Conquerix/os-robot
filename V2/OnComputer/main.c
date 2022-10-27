#include "coroutine.h"
#include "utils.h"
#include "mqtt.h"


#include "funcDefns.h"
#include "grid.c"
#include "file.c"
#include "node.c"
#include "path.c"

#include "mapToGrid.c"
//#include "mapToMatrix.c"
#include "posix_sockets.h"
#include "aroundEqual.c"
#include "cellIdConversion.c"
#include "cellPosConversions.c"
#include "positionToNode.c"
#include "fonctions_v2.c"
#include "robotToPointV1.c"
#include "tradPath.c"
#include "updateMap.c"
#include "updateObjectiveCell.c"
#include "mapFromFile.c"

#include "mqtt.c"
#include "mqtt_pal.c"





int alive;             /* Program is alive */

float us_distance;
int sonar_direction;
int gyro_angle;
int compass_direction;

int path_updated;

Position robot_position, sonar_position;

int path[V];
int map[HAUTEUR_GRILLE][LARGEUR_GRILLE];
intPoint objective_cell;

motorCommand wheel_command;

// Client is used globally by coroutines
struct mqtt_client client;

intPoint * pathList;

int left_wheel_pos, right_wheel_pos;
int robot_init, robot_uninit, robot_connected;

int ratio;

void publish_callback(void** unused, struct mqtt_response_publish *published){
  char* topic_name = (char*) malloc(published->topic_name_size + 1);
  memcpy(topic_name, published->topic_name, published->topic_name_size);
  topic_name[published->topic_name_size] = '\0';

  fprintf(stderr, "Received publish('%s'): %s\n", topic_name, (const char*) published->application_message);

  if        (strcmp(topic_name, "us_distance")       == 0){
    us_distance       = atof(published->application_message);

  } else if (strcmp(topic_name, "gyro_angle")        == 0){
    gyro_angle        = atoi(published->application_message);

  } else if (strcmp(topic_name, "compass_direction") == 0){
    compass_direction = atoi(published->application_message);

  } else if (strcmp(topic_name, "left_wheel_pos")    == 0){
    left_wheel_pos    = atoi(published->application_message);

  } else if (strcmp(topic_name, "right_wheel_pos")   == 0){
    right_wheel_pos   = atoi(published->application_message);

  } else if (strcmp(topic_name, "sonar_direction")   == 0){
    sonar_direction   = atoi(published->application_message);

  } else if (strcmp(topic_name, "robot_init")        == 0){
    robot_init        = 1;

  } else if (strcmp(topic_name, "robot_connected")   == 0){
    robot_connected   = 1;

  } else if (strcmp(topic_name, "robot_uninit")      == 0){
    robot_uninit      = 1;
  }

  free(topic_name);
}

void exit_example(int status, int sockfd, pthread_t *client_daemon){
  if (sockfd != -1) close(sockfd);
  if (client_daemon != NULL) pthread_cancel(*client_daemon);
  exit(status);
}

void* client_refresher(void* client){
  while(1){
    mqtt_sync((struct mqtt_client*) client);
    usleep(100000U);
  }
  return NULL;
}

CORO_CONTEXT( handle_robot_position );
CORO_CONTEXT( handle_map_changes );
CORO_CONTEXT( handle_path_update );
CORO_CONTEXT( where_to_go );


CORO_DEFINE( handle_robot_position ){

  CORO_LOCAL int left_wheel_mov, left_wheel_pos_old, right_wheel_mov, right_wheel_pos_old;

  CORO_BEGIN();
  for (;;){
    CORO_WAIT(left_wheel_pos != left_wheel_pos_old && right_wheel_pos != right_wheel_pos_old);
    left_wheel_mov      = getWheelAngleDifference(left_wheel_pos_old,  left_wheel_pos);
    right_wheel_mov     = getWheelAngleDifference(right_wheel_pos_old, right_wheel_pos);
    robot_position      = getNewRobotPos(robot_position, left_wheel_mov, right_wheel_mov);
    fprintf(stderr, "Robot Position: (%f - %f)\n", robot_position.x, robot_position.y);
    sonar_position      = getSonarPosition(robot_position, sonar_direction);
    left_wheel_pos_old  = left_wheel_pos;
    right_wheel_pos_old = right_wheel_pos;
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( handle_map_changes ){

  CORO_LOCAL Point watched_object_pos;

  CORO_BEGIN();
  for(;;){
    fprintf(stderr, "COROUTINE handle_map_changes\n");
    watched_object_pos = getWatchedObject(sonar_position, us_distance/10);
    updateMap(watched_object_pos, sonar_position, map);
    updateObjectiveCell(objective_cell, robot_position);
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( handle_path_update ){

  CORO_LOCAL int map_state[HAUTEUR_GRILLE][LARGEUR_GRILLE], i, j;
  CORO_LOCAL int openLength, closedLength, pathLength, whereInPath;
  CORO_LOCAL char grid[HAUTEUR_GRILLE][LARGEUR_GRILLE];
  CORO_LOCAL Node openGrid[HAUTEUR_GRILLE][LARGEUR_GRILLE], closedGrid[HAUTEUR_GRILLE][LARGEUR_GRILLE], pathGrid[HAUTEUR_GRILLE][LARGEUR_GRILLE];
  CORO_LOCAL Node start, goal, tmpNode;
  CORO_LOCAL int tmp1, tmp2;
  CORO_LOCAL intPoint objective_cell_state;


  CORO_BEGIN();
  path_updated = 0;
  objective_cell_state.x = 25;
  objective_cell_state.y = 30;

  for(;;){
    fprintf(stderr, "COROUTINE handle_path_update\n");
    CORO_WAIT( !mapsAreEqual(map, map_state) || objective_cell_state.x != objective_cell.x || objective_cell_state.y != objective_cell.y);

    openLength = 0;
    closedLength = 0;
    pathLength = 0;

    // Initializes grid
  	clearGrid(grid);
    mapToGrid(map, grid);
    grid[objective_cell.x][objective_cell.y] = 'G';

		// Initializing Node arrays
		initNodes(openGrid);
		initNodes(closedGrid);
		initNodes(pathGrid);


    clearNode(&start);
    clearNode(&goal);
		// Gets the node of start and goal
    tmpNode = positionToNode(robot_position);

    start.x = tmpNode.x * 2;
    start.y = tmpNode.y * 2;
    start.exists = tmpNode.exists;

    goal.x = objective_cell.y;
    goal.y = objective_cell.x;
    goal.exists = 1;

    addListNode(closedGrid, &closedLength, start);
    findPath(grid, openGrid, closedGrid, pathGrid, start, goal, start, &openLength, &closedLength, &pathLength);
    if(pathLength > 0) {
      if (pathList != NULL){
        free(pathList);
      }
      pathList = malloc(sizeof(int) * pathLength * 2);
      /*
      for (tmp1 = 0; tmp1 < HAUTEUR_GRILLE; tmp1++){
        for (tmp2 = 0; tmp2 < LARGEUR_GRILLE; tmp2++){
          if (pathGrid[tmp1][tmp2].exists){
            fprintf(stderr, "X");
          } else if (grid[tmp1][tmp2] == 'O'){
            fprintf(stderr, " ");
          } else {
            fprintf(stderr, "%c", grid[tmp1][tmp2]);
          }
        }
        fprintf(stderr, "\n");
      }
      */
      pathToPathList(pathGrid, pathList, whereInPath, pathLength, start, goal);
    }
    //writeFile(grid, "map.txt");

    path_updated++;
    for (i=0;i<HAUTEUR_GRILLE;i++){
      for (j=0;j<LARGEUR_GRILLE;j++){
        map_state[i][j] = map[i][j];
      }
    }
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( where_to_go ){

  CORO_LOCAL int where_in_path, where_to_go_path_state, distance_done, i;
  CORO_LOCAL intPoint cell_to_go;
  CORO_LOCAL Point pos_to_go;
  CORO_LOCAL char application_message_left[16], application_message_right[16];
  CORO_LOCAL motorCommand wheel_command_old;

  CORO_BEGIN();
  distance_done = 0;
  where_in_path = 0;
  where_to_go_path_state = 0;
  cell_to_go.x = -1;
  cell_to_go.y = -1;
  for(;;){
    if (pathList != NULL){
      if (path_updated > where_to_go_path_state) {
        where_to_go_path_state = path_updated;
        where_in_path = 0;
        distance_done = 0;
      }
      if (pathList[where_in_path].x != objective_cell.x || pathList[where_in_path].y != objective_cell.y){
        if (aroundEqual(robot_position.x, pos_to_go.x, 1) && aroundEqual(robot_position.y, pos_to_go.y, 1)){
          where_in_path += 5;
          i=0;
          for (i=where_in_path-4;i<where_in_path;i++){
            if (pathList[i].x == objective_cell.x || pathList[i].y == objective_cell.y){
              where_in_path = i;
              i += 5;
            }
          }
        }
        if (pathList[where_in_path].x != cell_to_go.x || pathList[where_in_path].y != cell_to_go.y){
          cell_to_go = pathList[where_in_path];
          pos_to_go = cellToPos(cell_to_go);
        }
      }

      wheel_command_old = wheel_command;
      wheel_command = robotToPointV1(robot_position, pos_to_go);
      if (wheel_command.left != wheel_command_old.left){
        fprintf(stderr, "New Left Wheel Command : %d\n", wheel_command.left);
        snprintf(application_message_left, sizeof(application_message_left), "%d", wheel_command.left);
        mqtt_publish(&client, "wheel_command.left", application_message_left, strlen(application_message_left) + 1, MQTT_PUBLISH_QOS_0);
      }
      if (wheel_command.right != wheel_command_old.right){
        fprintf(stderr, "New Right Wheel Command : %d\n", wheel_command.right);
        snprintf(application_message_right, sizeof(application_message_right), "%d", wheel_command.right);
        mqtt_publish(&client, "wheel_command.right", application_message_right, strlen(application_message_right) + 1, MQTT_PUBLISH_QOS_0);
      }
    }
    CORO_YIELD();
  }
  CORO_END();
}


int main( int argc, char *argv[] ) {

  const char* addr;
  const char* port;

  size.width  = LARGEUR_GRILLE;
  size.height = HAUTEUR_GRILLE;

  /* get address (argv[1] if present) */
  if (argc > 1) {
    addr = argv[1];
  } else {
    addr = "test.mosquitto.org";
  }

  /* get port number (argv[2] if present) */
  if (argc > 2){
    port = argv[2];
  } else {
    port = "1883";
  }

  int sockfd = open_nb_socket(addr, port);

  if (sockfd == -1) {
    perror("Failed to open socket: ");
    exit_example(EXIT_FAILURE, sockfd, NULL);
  }

  uint8_t sendbuf[2048];
  uint8_t recvbuf[1024];
  mqtt_init(&client, sockfd, sendbuf, sizeof(sendbuf), recvbuf, sizeof(recvbuf), publish_callback);
  const char* client_id = NULL;
  uint8_t connect_flags = MQTT_CONNECT_CLEAN_SESSION;
  mqtt_connect(&client, client_id, NULL, NULL, 0, NULL, NULL, connect_flags, 400);

  if (client.error != MQTT_OK){
    fprintf(stderr, "error: %s\n", mqtt_error_str(client.error));
    exit_example(EXIT_FAILURE, sockfd, NULL);
  }

  pthread_t client_daemon;

  if(pthread_create(&client_daemon, NULL, client_refresher, &client)){
    fprintf(stderr, "Failed to start client daemon.\n");
    exit_example(EXIT_FAILURE, sockfd, NULL);
  }

  mqtt_subscribe(&client, "us_distance",       0);
  mqtt_subscribe(&client, "gyro_angle",        0);
  mqtt_subscribe(&client, "compass_direction", 0);
  mqtt_subscribe(&client, "left_wheel_pos",    0);
  mqtt_subscribe(&client, "right_wheel_pos",   0);
  mqtt_subscribe(&client, "sonar_direction",   0);
  mqtt_subscribe(&client, "robot_init",        0);
  mqtt_subscribe(&client, "robot_connected",   0);
  mqtt_subscribe(&client, "robot_uninit",      0);

  robot_init      = 0;
  robot_connected = 0;
  robot_uninit    = 0;

  fprintf(stderr, "Waiting for the robot.\n");

  int i = 0;
  while (!robot_init && i <= 10){
    usleep(1000000U);
    fprintf(stderr,".");
    i++;
  }
  if (!robot_init){
    fprintf(stderr, "\nNo robot tried connecting.\nAborting...\n");
    exit_example(EXIT_FAILURE, sockfd, NULL);
  }

  fprintf(stderr, "A robot is present, sending a response.\n");
  mqtt_publish(&client, "computer_response", " ", 2, MQTT_PUBLISH_QOS_0);
  usleep(1000000U);

  fprintf(stderr, "Waiting for the answer of the robot.\n");
  i = 0;
  while (!robot_init && i <= 10){
    usleep(1000000U);
    fprintf(stderr,".");
    i++;
  }
  if (!robot_init){
    fprintf(stderr, "\nRobot did not answer.\nAborting...\n");
    exit_example(EXIT_FAILURE, sockfd, NULL);
  }
  fprintf(stderr, "Robot hooked correctly!\n");

  FILE *file = fopen("map.txt", "r");
  int errorMap = mapFromFile(file, map, &robot_position, &objective_cell);
  if (errorMap){fprintf(stderr, "ERROR WHILE READING MAP\n");}


  while ( !robot_uninit ) {
    CORO_CALL( handle_robot_position );
    CORO_CALL( handle_map_changes );
    CORO_CALL( handle_path_update );
    CORO_CALL( where_to_go );
    usleep(10000U);
  }

  printf("Robot disconnected, closing...\n");

  exit_example(EXIT_SUCCESS, sockfd, &client_daemon);
  return ( 0 );
}
