#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "coroutine.h"

#define HAUTEUR_CIRCUIT 200
#define LARGEUR_CIRCUIT 120
#define HAUTEUR_GRILLE 100
#define LARGEUR_GRILLE 60
#define V (HAUTEUR_GRILLE * LARGEUR_GRILLE)


int alive;             /* Program is alive */

float us_distance;
int sonar_direction;
int gyro_angle;
int compass_direction;

int path_updated;

Position robot_position, sonar_position;

int path[V];
int map[HAUTEUR_GRILLE][LARGEUR_GRILLE];
int objective_cell_id;

motorCommand wheel_command;

// Client is used globally by coroutines
struct mqtt_client client;

GridSize size;


int ratio;

void publish_callback(void** unused, struct mqtt_response_publish *published){
  char* topic_name = (char*) malloc(published->topic_name_size + 1);
  memcpy(topic_name, published->topic_name, published->topic_name_size);
  topic_name[published->topic_name_size] = '\0';

  printf("Received publish('%s'): %s\n", topic_name, (const char*) published->application_message);

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
CORO_CONTEXT( where_to_go );
CORO_CONTEXT( drive );


CORO_DEFINE( handle_robot_position ){

  CORO_LOCAL int left_wheel_mov, left_wheel_pos, left_wheel_pos_old, right_wheel_mov, right_wheel_pos, right_wheel_pos_old;

  CORO_BEGIN();
  left_wheel_pos_old  = left_wheel_pos;
  right_wheel_pos_old = right_wheel_pos;
  for (;;){
    CORO_WAIT(left_wheel_pos != left_wheel_pos_old && right_wheel_pos != right_wheel_pos_old);
    left_wheel_mov      = getWheelAngleDifference(left_wheel_pos_old,  left_wheel_pos);
    right_wheel_mov     = getWheelAngleDifference(right_wheel_pos_old, right_wheel_pos);
    robot_position      = getNewRobotPos(robot_position, left_wheel_mov, right_wheel_mov);
    sonar_position      = getSonarPos(robot_position, sonar_direction);
    left_wheel_pos_old  = left_wheel_pos;
    right_wheel_pos_old = right_wheel_pos;
  }
  CORO_END();
}

CORO_DEFINE( handle_map_changes ){

  CORO_LOCAL int map_state[HAUTEUR_GRILLE][LARGEUR_GRILLE];
  CORO_LOCAL Point watched_object_pos;
  CORO_LOCAL int openLength, closedLength, pathLength;
  CORO_LOCAL char grid[size.height][size.width];
  CORO_LOCAL Node open[size.height][size.width], closed[size.height][size.width], path[size.height][size.width];
  CORO_LOCAL Node start, goal;

  CORO_BEGIN();
  for(;;){
    openLength = 0;
    closedLength = 0;
    pathLength = 0;

  	// Initializes grid
  	clearGrid(grid);
    mapToGrid(map, grid);

  	// If there is no error reading grid from file
  	if(gridError == 0) {
  		struct timeb startTime, endTime;
    		int diff;
  		ftime(&startTime);

  		// Makes a copy of the grid for the solution
  		copyGrid(solvedGrid, grid);

  		// Prints the original map
  		printf("=== Original Grid ===\n");
  		printGrid(grid);


  		// Initializing Node arrays
  		initNodes(open);
  		initNodes(closed);
  		initNodes(path);


  		// Gets the node of start and goal
      start.x = ;
      start.y = h;
      start.exists = 1;

      goal.x = w;
      goal.y = h;
      goal.exists = 1;

  		// If start node was not found
  		if(start.exists == 0) {
  			printf("Error: Start node was not found.\n");
  		}
  		// If goal node was not found
  		if(goal.exists == 0) {
  			printf("Error: Goal node was not found.\n");
  		}

  		// If start and goal nodes both exist
  		if(start.exists == 1 && goal.exists == 1) {
  			// Add start node to the closed list
  			addListNode(closed, &closedLength, start);

  			findPath(grid, open, closed, path, start, goal, start, &openLength, &closedLength, &pathLength);

  			// If path was found..
  			if(pathLength > 0) {
  				// Inserts spaces into solvedGrid
  				insertPath(solvedGrid, path);

  				// Recording endtime, Calculating runtime (diff)
  				ftime(&endTime);
  	    		diff = (int) (1000.0 * (endTime.time - startTime.time) +
  				(endTime.millitm - startTime.millitm));

  				// Prints completion time
  	    		printf("\nSOLVED.. in %u milliseconds\n", diff);

  				// Prints solved grid
  				printGrid(solvedGrid);

  				// Writes solution file
  				writeFile(solvedGrid, argv[1]);
  			}
  		}
  	}
  }

    watched_object_pos = getWatchedObjectPos(sonar_position, sonar_direction, us_distance/10);
    updateMap(watched_object_pos, sonar_position, map);
    objective_cell_id = updateObjectiveCell(objective_cell_id, robot_position);
  }
  CORO_END();
}

CORO_DEFINE( handle_path_update ){

  CORO_LOCAL int map_state[HAUTEUR_GRILLE][LARGEUR_GRILLE], objective_cell_id_state, i, j;

  CORO_BEGIN();
  path_updated = 0;
  objective_cell_id_state = (25*60+30);
  for(;;){
    CORO_WAIT( !mapsAreEqual(map, map_state) || objective_cell_id_state != objective_cell_id);
    for (i=0;i<HAUTEUR_GRILLE;i++){
      for (j=0;j<LARGEUR_GRILLE;j++){
        map_state[i][j] = map[i][j];
      }
    }

    path_updated++;
  }
  CORO_END();
}

CORO_DEFINE( where_to_go ){

  CORO_LOCAL int where_in_path, where_to_go_path_state, cell_to_go_id, distance_done, i;
  CORO_LOCAL intPoint cell_to_go;
  CORO_LOCAL Point pos_to_go;
  CORO_LOCAL char application_message_left[16], application_message_right[16];

  CORO_BEGIN();
  distance_done = 0;
  where_in_path = 0;
  where_to_go_path_state = 0;
  cell_to_go_id = -1;
  for(;;){

    if (path_updated > where_to_go_path_state) {
      where_to_go_path_state = path_updated;
      where_in_path = 0;
      distance_done = 0;
    }
    if (path[where_in_path] != objective_cell_id){
      if (aroundEqual(robot_position.x, pos_to_go.x, 1) && aroundEqual(robot_position.y, pos_to_go.y, 1)){
        where_in_path += 5;
        i=0;
        for (i=where_in_path-4;i<where_in_path;i++){
          if (path[i] == objective_cell_id){
            where_in_path = i;
            i += 5;
          }
        }
      }
      if (path[where_in_path] != cell_to_go_id){
        cell_to_go_id = path[where_in_path];
        cell_to_go = cellIdConversion(cell_to_go_id);
        pos_to_go = cellToPos(cell_to_go);
      }
    }

    wheel_command = robotToPointV1(robot_position, pos_to_go);
    snprintf(application_message_left, sizeof(application_message_left), "%d", wheel_command.left);
    mqtt_publish(&client, "us_distance", application_message_left, strlen(application_message_left) + 1, MQTT_PUBLISH_QOS_0);
    snprintf(application_message_right, sizeof(application_message_right), "%d", wheel_command.right);
    mqtt_publish(&client, "us_distance", application_message_right, strlen(application_message_right) + 1, MQTT_PUBLISH_QOS_0);
  }
  CORO_END();
}


int main( int argc, char *argv[] ) {

  const char* addr;
  const char* port;
  const char* topic;


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
  int i = 0;
  while (!robot_init && i <= 1000000){i++;}
  if (!robot_init){
    fprintf(stderr, "No robot tried connecting.\nAborting...\n");
    exit_example(EXIT_FAILURE, sockfd, NULL);
  }

  mqtt_publish(&client, "computer_response", " ", 2, MQTT_PUBLISH_QOS_0);

  i = 0;
  while (!robot_connected && i <= 1000000){i++;}
  if (!robot_init){
    fprintf(stderr, "Robot initialized but did not connect.\nAborting...\n");
    exit_example(EXIT_FAILURE, sockfd, NULL);
  }
  while ( !robot_uninit ) {
    CORO_CALL( handle_robot_position );
    CORO_CALL( handle_map_changes );
    CORO_CALL( where_to_go );
    CORO_CALL( drive );
    sleep_ms( 10 );
  }

  printf("Robot disconnected, closing...\n")

  exit_example(EXIT_SUCCESS, sockfd, &client_daemon);
  return ( 0 );
}
