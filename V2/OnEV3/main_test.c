#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "mqtt.h"
#include "coroutine.h"
#include "brick.h"

#include "posix_sockets.h"

#include "mqtt.c"
#include "mqtt_pal.c"

#define SPEED_LINEAR    40  /* Motor speed for linear motion, in percents */
#define SPEED_CIRCULAR  50  /* ... for circular motion */
int max_speed;         /* Motor maximal speed (will be detected) */
int wheel_speed;
#define MOTOR_LEFT     OUTD
#define MOTOR_RIGHT    OUTA
#define MOTOR_BOTH     ( MOTOR_LEFT | MOTOR_RIGHT )

// US Distance between 0-2550 (in cm)

#define US_CONTINUOUS  LEGO_EV3_US_US_DIST_CM
#define US_SINGLE      LEGO_EV3_US_US_SI_CM
#define US_LISTEN      LEGO_EV3_US_US_LISTEN  //Presence (0 or 1)

#define MOTOR_SONAR    OUTC
#define US_ROT_SPEED    5
int sonar_max_speed;

// GYRO Angle between -32768 and 32767

#define GYRO_ANG       LEGO_EV3_GYRO_GYRO_ANG

// COMPASS Direction between 0 and 359

#define COMPASS_DIRECTION HT_NXT_COMPASS_COMPASS

#define HAUTEUR_CIRCUIT 200
#define LARGEUR_CIRCUIT 120
#define HAUTEUR_GRILLE 100
#define LARGEUR_GRILLE 60
#define V (HAUTEUR_GRILLE * LARGEUR_GRILLE)


// For manual command

int alive;             /* Program is alive */

int circuit_angle = 0;
int circuit_direction = 0;

// Sensors Addresses

POOL_T us;
POOL_T gyro;
POOL_T compass;

// Global Variables used between coroutines

float us_distance, us_distance_old;
int gyro_angle, gyro_angle_old, gyro_angle_uncomputed;
int compass_direction, compass_direction_old, compass_direction_uncomputed;
int left_wheel_pos, right_wheel_pos, sonar_direction;

float robot_x, robot_y;
int sonar_position;

int path[V];
int objective_cell_id;

int computer_found;

typedef struct motorCommand{
  int left;
  int right;
} motorCommand;

motorCommand wheel_command;

// Map global variables

int ratio;

// Client is used globally by coroutines
struct mqtt_client client;

int init( void ) {
    if ( tacho_is_plugged( MOTOR_BOTH, TACHO_TYPE__NONE_ )) {  /* any type of motor */
        max_speed = tacho_get_max_speed( MOTOR_LEFT, 0 );
        tacho_reset( MOTOR_BOTH );
        printf("--- Motors Found ! --- \n");
        printf("--- Left motor should be on port D and right motor on port A ---\n");
    } else {
        printf("-/!\\- At least one motor was not found ! -/!\\-\n");
        printf("--- Left motor should be on port D and right motor on port A ---\n");
        printf("-/!\\- Aborting... -/!\\-\n");
        /* Inoperative without motors */
        return ( 0 );
    }
    if ( tacho_is_plugged( MOTOR_SONAR, TACHO_TYPE__NONE_ )) {  /* any type of motor */
        tacho_reset( MOTOR_SONAR );
        printf("--- Sonar Motor Found ! --- \n");
    } else {
        printf("-/!\\- Sonar Motor not found ! -/!\\-\n");
        printf("--- It should be on port C ---\n");
        printf("-/!\\- Aborting... -/!\\-\n");
        /* Inoperative without motors */
        return ( 0 );
    }
    us = sensor_search( LEGO_EV3_US );
    if (us) {
      printf("--- Sonar Found !! ---\n");
      sensor_set_mode(us, US_CONTINUOUS);

    } else {printf("-/!\\- Sonar Not Found !! -/!\\-\n");}

    gyro = sensor_search( LEGO_EV3_GYRO );
    if (gyro) {
      printf("--- Gyro Found !! ---\n");
      sensor_set_mode(gyro, GYRO_ANG);
      circuit_angle = sensor_get_value0(gyro, 0);
    } else {printf("-/!\\- Gyro Not Found !! -/!\\-\n");}

    compass = sensor_search( HT_NXT_COMPASS );
    if (compass) {
      printf("--- Compass Found !! ---\n");
      sensor_set_mode(compass, COMPASS_DIRECTION);
      circuit_direction = sensor_get_value0(compass, 0);
    } else {printf("-/!\\- Compass Not Found !! -/!\\-\n");}

    /*
    if (!(us && gyro && compass)) {
      printf("-/!\\- Some sensors were not found -/!\\-\n");
      printf("--- Please use the brick buttons ---\n");
    }
    */


    printf( "Press BACK on the EV3 brick for EXIT...\n" );
    return ( 1 );
}

void publish_callback(void** unused, struct mqtt_response_publish *published){
  char* topic_name = (char*) malloc(published->topic_name_size + 1);
  memcpy(topic_name, published->topic_name, published->topic_name_size);
  topic_name[published->topic_name_size] = '\0';

  fprintf(stderr, "Received publish('%s'): %s\n", topic_name, (const char*) published->application_message);

  if (strcmp(topic_name, "wheel_command.left") == 0){
    wheel_command.left = atoi(published->application_message);
  } else if (strcmp(topic_name, "wheel_command.right") == 0){
    wheel_command.right = atoi(published->application_message);
  } else if (strcmp(topic_name, "computer_response") == 0){
    computer_found = 1;
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





void handle_gyro_measures (){

  char application_message[6];

  gyro_angle_old = gyro_angle;
  gyro_angle_uncomputed = sensor_get_value0(gyro, 0);
  fprintf(stderr, "Gyro angle = %d\n", gyro_angle_uncomputed);
  gyro_angle = gyro_angle_uncomputed - circuit_angle;
  gyro_angle += 180;
  gyro_angle = gyro_angle % 360;
  if (gyro_angle < 0) {compass_direction += 360;}
  fprintf(stderr, "Compass direction after computation = %d\n", gyro_angle);
  if (gyro_angle != gyro_angle_old){
    snprintf(application_message, sizeof(application_message), "%d", gyro_angle);
    mqtt_publish(&client, "gyro_angle", application_message, strlen(application_message) + 1, MQTT_PUBLISH_QOS_0);
  }
}

void handle_compass_measures(){

  char application_message[6];

  compass_direction_old = compass_direction;
  compass_direction_uncomputed = sensor_get_value0(compass, 0);
  fprintf(stderr, "Compass direction = %d - ", compass_direction);
  compass_direction = compass_direction_uncomputed - circuit_direction;
  compass_direction += 360;
  compass_direction = compass_direction % 360;
  fprintf(stderr, "Compass direction after computation = %d\n", compass_direction);
  if (compass_direction != compass_direction_old){
    snprintf(application_message, sizeof(application_message), "%d", compass_direction);
    mqtt_publish(&client, "compass_direction", application_message, strlen(application_message) + 1, MQTT_PUBLISH_QOS_0);
  }
}
void handle_sonar_measures(){

  char application_message[16];

  us_distance_old = us_distance;
  us_distance = sensor_get_value0(us, 0);
  if (us_distance != us_distance_old) {
    snprintf(application_message, sizeof(application_message), "%.6f", us_distance);
    mqtt_publish(&client, "us_distance", application_message, strlen(application_message) + 1, MQTT_PUBLISH_QOS_0);
  }
}
void handle_motor_positions(){

  char application_message_left[6], application_message_right[6], application_message_sonar[6];
  int left_wheel_pos_old, right_wheel_pos_old, sonar_direction_old;

  left_wheel_pos_old = left_wheel_pos;
  left_wheel_pos  = tacho_get_position(MOTOR_LEFT,  0);
  if (left_wheel_pos != left_wheel_pos_old){
    snprintf(application_message_left, sizeof(application_message_left), "%d", left_wheel_pos);
    mqtt_publish(&client, "left_wheel_pos", application_message_left, strlen(application_message_left) + 1, MQTT_PUBLISH_QOS_0);
  }

  right_wheel_pos_old = right_wheel_pos;
  right_wheel_pos = tacho_get_position(MOTOR_RIGHT, 0);
  if (right_wheel_pos != right_wheel_pos_old){
    snprintf(application_message_right, sizeof(application_message_right), "%d", right_wheel_pos);
    mqtt_publish(&client, "right_wheel_pos", application_message_right, strlen(application_message_right) + 1, MQTT_PUBLISH_QOS_0);
  }

  sonar_direction_old = sonar_direction;
  sonar_direction = tacho_get_position(MOTOR_SONAR, 0);
  if (sonar_direction != sonar_direction_old){
    snprintf(application_message_sonar, sizeof(application_message_sonar), "%d", sonar_direction);
    mqtt_publish(&client, "sonar_direction", application_message_sonar, strlen(application_message_sonar) + 1, MQTT_PUBLISH_QOS_0);
  }
}

CORO_CONTEXT( race_stop );
CORO_CONTEXT( sonar_movement );
CORO_CONTEXT( drive );

CORO_DEFINE( race_stop ) {
    CORO_LOCAL uint8_t keys, pressed = EV3_KEY__NONE_;
    CORO_BEGIN();
    for ( ; ; ) {
        /* Waiting any brick's key is pressed or released */
        CORO_WAIT(( keys = brick_keys()) != pressed );
        pressed = keys;
        if (pressed == EV3_KEY_BACK){
            wheel_command.left  = 0;
            wheel_command.right = 0;
            alive = 0;
            mqtt_publish(&client, "robot_uninit", " ", 2, MQTT_PUBLISH_QOS_0);
        }
        CORO_YIELD();
    }
    CORO_END();
}

CORO_DEFINE( sonar_movement ){
  CORO_BEGIN();
  tacho_set_speed_sp( MOTOR_SONAR, 5 * max_speed / 100);
  tacho_set_position_sp( MOTOR_SONAR, 20);
  for (;;){
    if (sonar_direction >= 10) {
      tacho_set_position_sp( MOTOR_SONAR, -20);
    } else if (sonar_direction <= -10) {
      tacho_set_position_sp( MOTOR_SONAR, 20);
    }
    tacho_run_to_abs_pos(MOTOR_SONAR);
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( drive ){

  CORO_LOCAL motorCommand motor_state;

  CORO_BEGIN();
  motor_state.left = 0;
  motor_state.right = 0;
  for (;;){
    CORO_WAIT((motor_state.left != wheel_command.left) || (motor_state.right != wheel_command.right));
	   fprintf(stderr, "Left motor : %d | Right motor : %d\n", wheel_command.left, wheel_command.right);
	   tacho_set_speed_sp( MOTOR_LEFT, wheel_command.left * max_speed / 100);
	   tacho_set_speed_sp( MOTOR_RIGHT, wheel_command.right * max_speed / 100);
	   tacho_run_forever( MOTOR_BOTH );
	   motor_state = wheel_command;
     CORO_YIELD();
  }
  CORO_END();
}


int main( int argc, char *argv[] ) {

    const char* addr;
    const char* port;
    const char* topic;
    int i;

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

    printf( "Waiting the EV3 brick online...\n" );
    if ( !brick_init()) return ( 1 );
    usleep(1000000U);
    printf( "*** ( EV3 ) Hello! ***\n" );
    alive = init();

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
    fprintf(stderr, "Connecting to server\n");
    mqtt_connect(&client, client_id, NULL, NULL, 0, NULL, NULL, connect_flags, 400);
    fprintf(stderr, "Connected to server\n");
    if (client.error != MQTT_OK){
      fprintf(stderr, "error: %s\n", mqtt_error_str(client.error));
      exit_example(EXIT_FAILURE, sockfd, NULL);
    }

    pthread_t client_daemon;

    if(pthread_create(&client_daemon, NULL, client_refresher, &client)){
      fprintf(stderr, "Failed to start client daemon.\n");
      exit_example(EXIT_FAILURE, sockfd, NULL);
    }

    mqtt_subscribe(&client, "wheel_command.left",  0);
    mqtt_subscribe(&client, "wheel_command.right", 0);
    mqtt_subscribe(&client, "computer_response",   0);

    computer_found = 0;
    fprintf(stderr, "Sending request to the computer.\n");
    mqtt_publish(&client, "robot_init", " ", 2, MQTT_PUBLISH_QOS_0);
    usleep(1000000U);
    i = 0;
    while (!computer_found && i <= 10){
      usleep(1000000U);
      fprintf(stderr,".");
      i++;
    }
    if (!computer_found){
      fprintf(stderr, "\nRobot initialized but no computer found.\nAborting...\n");
      exit_example(EXIT_FAILURE, sockfd, NULL);
    }
    fprintf(stderr, "\nComputer found !! Sending back response\n");
    mqtt_publish(&client, "robot_connected", " ", 2, MQTT_PUBLISH_QOS_0);
    usleep(1000000U);

    while ( alive ) {
      handle_sonar_measures();
      handle_gyro_measures();
      handle_compass_measures();
      handle_motor_positions();
      CORO_CALL( race_stop );
      CORO_CALL( sonar_movement );
      CORO_CALL( drive );
      sleep_ms( 10 );
    }

    printf("--- Sending message to stop computer ---\n");
    mqtt_publish(&client, "robot_uninit", " ", 2, MQTT_PUBLISH_QOS_0);

    brick_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );

    exit_example(EXIT_SUCCESS, sockfd, &client_daemon);
    return ( 0 );
}
