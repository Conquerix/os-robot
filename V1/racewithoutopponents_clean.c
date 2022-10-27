#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "coroutine.h"
#include "brick.h"
#define SPEED_LINEAR    50  /* Motor speed for linear motion, in percents */
#define SPEED_CIRCULAR  50  /* ... for circular motion */
int max_speed;         /* Motor maximal speed (will be detected) */
#define MOTOR_LEFT     OUTD
#define MOTOR_RIGHT    OUTA
#define MOTOR_BOTH     ( MOTOR_LEFT | MOTOR_RIGHT )

// US Distance between 0-2550 (in cm)

#define US_CONTINUOUS  LEGO_EV3_US_US_DIST_CM
#define US_SINGLE      LEGO_EV3_US_US_SI_CM
#define US_LISTEN      LEGO_EV3_US_US_LISTEN  //Presence (0 or 1)

// GYRO Angle between -32768 and 32767

#define GYRO_ANG       LEGO_EV3_GYRO_GYRO_ANG

// COMPASS Direction between 0 and 359

#define COMPASS_DIRECTION HT_NXT_COMPASS_COMPASS


// For manual command

enum {
    STOP,
    FORTH,
    BACK,
    LEFT,
    RIGHT,
};

int command = STOP;    /* Command for `drive` coroutine */

int alive;             /* Program is alive */

int manual = 0; //the robot can be controlled manually
int circuit_direction = 0;

// Sensors Addresses

POOL_T us;
POOL_T gyro;
POOL_T compass;

// Global Variables used between coroutines

float us_distance, us_distance_old, us_distance_expected;
int gyro_angle, gyro_angle_old, gyro_angle_expected;
int gyroscope_direction, gyroscope_direction_old, gyroscope_direction_expected;

int gyroscope_direction_uncomputed;

int close_to_middle_wall = 0;

// Map global variables : The different zones of the map

enum {
    MIDDLE_RIGHT,
    UP_RIGHT,
    UP_LEFT,
    MIDDLE_LEFT,
    BOTTOM_LEFT,
    BOTTOM_RIGHT,
};

int position_in_circuit = MIDDLE_RIGHT, position_in_circuit_old = BOTTOM_RIGHT;
int ratio;
float us_distance_to_main_wall = 300;
float us_distance_to_middle_wall = 300;

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
    us = sensor_search( LEGO_EV3_US );
    if (us) {
      printf("--- Sonar Found !! ---\n");
      sensor_set_mode(us, US_CONTINUOUS);

    } else {printf("-/!\\- Sonar Not Found !! -/!\\-\n");}

    gyro = sensor_search( LEGO_EV3_GYRO );
    if (gyro) {
      printf("--- Gyro Found !! ---\n");
      sensor_set_mode(gyro, GYRO_ANG);
    } else {printf("-/!\\- Gyro Not Found !! -/!\\-\n");}

    compass = sensor_search( HT_NXT_COMPASS );
    if (compass) {
      printf("--- Compass Found !! ---\n");
      sensor_set_mode(compass, COMPASS_DIRECTION);
      circuit_direction = sensor_get_value0(compass, 0);
    } else {printf("-/!\\- Compass Not Found !! -/!\\-\n");}

    if (!(us && compass)) {
      printf("-/!\\- Some sensors were not found -/!\\-\n");
      printf("--- Please use the brick buttons ---\n");
      manual = 1;
    }

    printf( "Press BACK on the EV3 brick for EXIT...\n" );
    return ( 1 );
}

// ray is the ray of the trajectory we want in cm
int calcul_ratio(double ray) {
    
    double ratio;
    double a;
	a = 13; //distance between the wheels
	ratio = (ray-(a/2))/(ray+a/2) * 100;
    printf("Le rapport de vitesses des roues est %f\n", ratio);
	return ratio;
}


void map_middle_right(){
  if (position_in_circuit != position_in_circuit_old){
    ratio = 40;
    //depending on the orientation of the robot, it knows when to turn and how much
    if (gyroscope_direction > 3  && gyroscope_direction < 180){
      command = LEFT;
      printf("Test5.2.1 - ");
    } else if (gyroscope_direction >= 180  && gyroscope_direction < 357){
      command = RIGHT;
      printf("Test5.2.2 - ");
    } else {
      command = FORTH;
      printf("Test5.2.3 - ");
      position_in_circuit_old = position_in_circuit;
    }
  } else {
    if (us_distance <= us_distance_to_main_wall){
      //When the robot is close enough to the wall, it means it has entered the UP_RIGHT zone
      position_in_circuit = UP_RIGHT;
      circuit_direction = gyroscope_direction_uncomputed; //gyroscope calibration
      command = LEFT;
      printf("Test5.2.4 - ");
      ratio = 40; //calcul_ratio(30);
      printf("Ratio = %d\n", ratio);
    }
  }
  return;
}

//Each zone of the map has its own function that the robot uses to get through the zone
//The method for each is similar to map_middle_left

void map_up_right(){
  if (gyroscope_direction > 266 && gyroscope_direction < 274){
    position_in_circuit = UP_LEFT;
    //circuit_direction = (gyroscope_direction_uncomputed + 90) % 360; //gyroscope calibration
    command = FORTH;
    ratio = 40;
  }
  return;
}

void map_up_left(){
  if (position_in_circuit != position_in_circuit_old){
    if (us_distance <= us_distance_to_main_wall){
      position_in_circuit_old = position_in_circuit;
      command = LEFT;
      ratio = 40;//calcul_ratio(30);
    }
  } else if (gyroscope_direction > 176 && gyroscope_direction < 184){
    position_in_circuit = MIDDLE_LEFT;
    //circuit_direction = (gyroscope_direction_uncomputed + 180) % 360; //gyroscope calibration
  }
  return;
}

void map_middle_left(){
  if (us_distance <= us_distance_to_middle_wall) {
    close_to_middle_wall = 1;
  }
  if (position_in_circuit != position_in_circuit_old){
    ratio = 60;
    if (gyroscope_direction > 169  && gyroscope_direction < 349){
      command = LEFT;
    } else if (gyroscope_direction < 161  || gyroscope_direction > 349){
      command = RIGHT;
    } else {
      command = FORTH;
      position_in_circuit_old = position_in_circuit;
    }
  } else if (close_to_middle_wall){
    ratio = 60;
    if (gyroscope_direction < 161){
      command = RIGHT;
    } else if (gyroscope_direction > 169){
      command = LEFT;
    } else {
      command = FORTH;
      position_in_circuit = BOTTOM_LEFT;
      //circuit_direction = (gyroscope_direction_uncomputed + 180) % 360; //gyroscope calibration
      close_to_middle_wall = 0;
    }
  }
  return;
}

void map_bottom_left(){
  if (position_in_circuit != position_in_circuit_old){
    if (us_distance <= us_distance_to_main_wall){
      position_in_circuit_old = position_in_circuit;
      command = LEFT;
      ratio = 40; //calcul_ratio(30);
    }
  } else if (gyroscope_direction > 86 && gyroscope_direction < 94){
    position_in_circuit = BOTTOM_RIGHT;
    //circuit_direction = (gyroscope_direction_uncomputed + 270) % 360; //gyroscope calibration
  }
  return;
}

void map_bottom_right(){
  if (gyroscope_direction < 4  || gyroscope_direction > 356){
    position_in_circuit = MIDDLE_RIGHT;
    //circuit_direction = gyroscope_direction_uncomputed; //gyroscope calibration
  }
  return;
}

CORO_CONTEXT( manual_command );
CORO_CONTEXT( manual_drive );

CORO_CONTEXT( race_stop );
CORO_CONTEXT( handle_sonar_measures );
CORO_CONTEXT( handle_gyro_measures );
CORO_CONTEXT( handle_gyroscope_measures );
CORO_CONTEXT( race_map );
CORO_CONTEXT( race_drive );


CORO_DEFINE( manual_command ) {
    CORO_LOCAL uint8_t keys, pressed = EV3_KEY__NONE_;
    CORO_BEGIN();
    for ( ; ; ) {
        /* Waiting any brick's key is pressed or released */
        CORO_WAIT(( keys = brick_keys()) != pressed );
        pressed = keys;
        switch ( pressed ) {
        /* Quit */
        case EV3_KEY_BACK:
            command = STOP;
            alive = 0;
            break;
        /* Stop */
        case EV3_KEY__NONE_:
        case EV3_KEY_CENTER:
            command = STOP;
            break;
        /* Forward */
        case EV3_KEY_DOWN:
            command = FORTH;
            break;
        /* Backward */
        case EV3_KEY_UP:
            command = BACK;
            break;
        /* Left */
        case EV3_KEY_RIGHT:
            command = LEFT;
            break;
        /* Right */
        case EV3_KEY_LEFT:
            command = RIGHT;
            break;
        }
        CORO_YIELD();
    }
    CORO_END();
}

CORO_DEFINE( manual_drive ) {
    CORO_LOCAL int speed_linear, speed_circular;
    CORO_LOCAL int state = STOP;
    CORO_BEGIN();
    speed_linear = max_speed * SPEED_LINEAR / 100;
    speed_circular = max_speed * SPEED_CIRCULAR / 100;
    for ( ; ; ) {
        /* Waiting new command */
        CORO_WAIT( state != command );
        switch ( command ) {
        case STOP:
            tacho_stop( MOTOR_BOTH );
            /* Waiting the vehicle is stopped */
            CORO_WAIT( !tacho_is_running( MOTOR_BOTH ));
            break;
        case FORTH:
            tacho_set_speed_sp( MOTOR_BOTH, speed_linear );
            tacho_run_forever( MOTOR_BOTH );
            break;
        case BACK:
            tacho_set_speed_sp( MOTOR_BOTH, -speed_linear );
            tacho_run_forever( MOTOR_BOTH );
            break;
        case LEFT:
            tacho_set_speed_sp( MOTOR_LEFT, -speed_circular );
            tacho_set_speed_sp( MOTOR_RIGHT, speed_circular );
            tacho_run_forever( MOTOR_BOTH );
            break;
        case RIGHT:
            tacho_set_speed_sp( MOTOR_LEFT, speed_circular );
            tacho_set_speed_sp( MOTOR_RIGHT, -speed_circular );
            tacho_run_forever( MOTOR_BOTH );
            break;
        }
        state = command;
    }
    CORO_END();
}

CORO_DEFINE( race_stop ) {
    CORO_LOCAL uint8_t keys, pressed = EV3_KEY__NONE_;
    CORO_BEGIN();
    for ( ; ; ) {
        /* Waiting any brick's key is pressed or released */
        CORO_WAIT(( keys = brick_keys()) != pressed );
        pressed = keys;
        if (pressed = EV3_KEY_BACK){
            command = STOP;
            alive = 0;
        }
        CORO_YIELD();
    }
    CORO_END();
}

CORO_DEFINE( handle_sonar_measures ) {

  CORO_BEGIN();
  printf("Test2.1 - ");
  if ( us == SOCKET__NONE_ ) {CORO_QUIT();}
  for(;;){
    printf("Test2.2 - ");
    us_distance_old = us_distance;
    us_distance = sensor_get_value0(us, 0);
    printf("Sonar distance = %f\n", us_distance);
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( handle_gyro_measures ) {

  CORO_BEGIN();
  printf("Test3.1 - ");
  if ( gyro == SOCKET__NONE_ ) {CORO_QUIT();}
  for (;;){
    printf("Test3.2 - ");
    gyro_angle_old = gyro_angle;
    gyro_angle = sensor_get_value0(gyro, 0);
    printf("Gyro angle = %d\n", gyro_angle);
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( handle_gyroscope_measures ) {

  CORO_BEGIN();
  printf("Test4.1 - ");
  if ( compass == SOCKET__NONE_ ) {CORO_QUIT();}
  for (;;){
    printf("Test4.2 - ");
    gyroscope_direction_old = gyroscope_direction;
    gyroscope_direction_uncomputed = sensor_get_value0(compass, 0);
    printf("gyroscope direction = %d - ", gyroscope_direction);
    gyroscope_direction = gyroscope_direction_uncomputed - circuit_direction;
    gyroscope_direction += 360;
    gyroscope_direction = gyroscope_direction % 360;
    printf("gyroscope direction after computation = %d\n", gyroscope_direction);
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( race_map ) {
  CORO_LOCAL int ray;
  CORO_BEGIN();
  for(;;){
    printf("Test5.1 - ");
    switch(position_in_circuit){
    case MIDDLE_RIGHT:
      printf("Test5.2 - ");
      map_middle_right();
      break;
    case UP_RIGHT:
      printf("Test5.3 - ");
      map_up_right();
      break;
    case UP_LEFT:
      printf("Test5.4 - ");
      map_up_left();
      break;
    case MIDDLE_LEFT:
      printf("Test5.5 - ");
      map_middle_left();
      break;
    case BOTTOM_LEFT:
      printf("Test5.6 - ");
      map_bottom_left();
      break;
    case BOTTOM_RIGHT:
      printf("Test5.7\n");
      map_bottom_right();
      break;
    }
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( race_drive ) {
  CORO_LOCAL int speed_exterior, speed_interior;
  CORO_LOCAL int state = STOP;
  CORO_LOCAL int ratio_state = 1;
  CORO_BEGIN();
  printf("Test6.1 - ");
  speed_exterior = max_speed * SPEED_LINEAR / 100;
  speed_interior = speed_exterior * ratio / 100;
  for ( ; ; ) {
      /* Waiting new command */
      CORO_WAIT( state != command || ratio_state != ratio);
      switch ( command ) {
      case STOP:
          printf("Test6.2 - ");
          tacho_stop( MOTOR_BOTH );
          /* Waiting the vehicle is stopped */
          CORO_WAIT( !tacho_is_running( MOTOR_BOTH ));
          break;
      case FORTH:
          printf("Test6.3 - ");
          tacho_set_speed_sp( MOTOR_BOTH, speed_exterior );
          tacho_run_forever( MOTOR_BOTH );
          break;
      case BACK:
          printf("Test6.4 - ");
          tacho_set_speed_sp( MOTOR_BOTH, -speed_exterior );
          tacho_run_forever( MOTOR_BOTH );
          break;
      case LEFT:
          printf("Test6.5 - ");
          tacho_set_speed_sp( MOTOR_LEFT, speed_interior );
          tacho_set_speed_sp( MOTOR_RIGHT, speed_exterior );
          tacho_run_forever( MOTOR_BOTH );
          break;
      case RIGHT:
          printf("Test6.6\n");
          tacho_set_speed_sp( MOTOR_LEFT, speed_exterior );
          tacho_set_speed_sp( MOTOR_RIGHT, speed_interior );
          tacho_run_forever( MOTOR_BOTH );
          break;
      }
      state = command;
      ratio_state = ratio;
  }
  CORO_END();

}

int main( int argc, char *argv[] ) {

    int manual = 0;
    if (argc > 1) {
      if (argv[1][0] == '1'){
        printf("Help message to be written here\n");
      } else if (argv[1][0] == '2') {
        manual = 1;
      }
    }
    printf( "Waiting the EV3 brick online...\n" );
    if ( !brick_init()) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
    alive = init();
    printf("Test1");
    if (manual) {
      while ( alive ) {
          CORO_CALL( handle_sonar_measures );
          CORO_CALL( handle_gyro_measures );
          CORO_CALL( handle_gyroscope_measures );
          CORO_CALL( manual_command );
          CORO_CALL( manual_drive );
          sleep_ms( 10 );
      }
    } else {
      while ( alive ) {
          CORO_CALL( race_stop );
          printf("Test2");
          CORO_CALL( handle_sonar_measures );
          printf("Test3");
          CORO_CALL( handle_gyro_measures );
          printf("Test4");
          CORO_CALL( handle_gyroscope_measures );
          printf("Test5");
          CORO_CALL( race_map );
          printf("Test6");
          CORO_CALL( race_drive );
          printf("Test7");
          sleep_ms( 10 );
      }
    }
    brick_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    return ( 0 );
}
