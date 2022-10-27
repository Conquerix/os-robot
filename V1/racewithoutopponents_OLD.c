#include <stdio.h>
#include <stdlib.h>
#include "coroutine.h"
#include "brick.h"
#define SPEED_LINEAR    75  /* Motor speed for linear motion, in percents */
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

// Sensors Addresses

POOL_T us;
POOL_T gyro;
POOL_T compass;

// Global Variables used between coroutines

float us_distance, us_distance_old, us_distance_expected;
int gyro_angle, gyro_angle_old, gyro_angle_expected;
int compass_direction, compass_direction_old, compass_direction_expected;

// Map global variables

enum {
    MIDDLE_RIGHT,
    UP_RIGHT,
    UP_LEFT,
    MIDDLE_LEFT,
    BOTTOM_LEFT,
    BOTTOM_RIGHT,
};

int position_in_circuit, position_in_circuit_old;
int ratio;
float us_distance_to_main_wall;
float us_distance_to_middle_wall;

int init( void ) {
    if ( tacho_is_plugged( MOTOR_BOTH, TACHO_TYPE__NONE_ )) {  /* any type of motor */
        max_speed = tacho_get_max_speed( MOTOR_LEFT, 0 );
        tacho_reset( MOTOR_BOTH );
        printf("--- Motors Found ! --- \n");
        printf("--- Left motor should be on port D and right motor on port A ---\n");
    } else {
        printf("-/!\- At least one motor was not found ! ---\n");
        printf("--- Left motor should be on port D and right motor on port A ---\n");
        printf("-/!\- Aborting... -/!\-\n");
        /* Inoperative without motors */
        return ( 0 );
    }
    us = sensor_search( LEGO_EV3_US );
    if (us) {
      printf("--- Sonar Found !! ---\n");
      sensor_set_mode(us, US_CONTINUOUS);
    } else {printf("-/!\- Sonar Not Found !! -/!\-\n");}

    gyro = sensor_search( LEGO_EV3_GYRO );
    if (gyro) {
      printf("--- Gyro Found !! ---\n");
      sensor_set_mode(gyro, GYRO_ANG);
    } else {printf("-/!\- Gyro Not Found !! -/!\-\n");}

    compass = sensor_search( HT_NXT_COMPASS );
    if (compass) {
      printf("--- Compass Found !! ---\n");
      sensor_set_mode(compass, COMPASS_DIRECTION);
    } else {printf("-/!\- Compass Not Found !! -/!\-\n");}

    if (!(us && gyro && compass)) {
      printf("-/!\- Some sensors were not found -/!\-\n");
      printf("--- Please use the brick buttons ---\n");
    }

    printf( "Press BACK on the EV3 brick for EXIT...\n" );
    return ( 1 );
}

/* prend en argument le rayon de courbure en cm*/
int calcul_ratio(double ray) {
    /* prend en argument le rayon de courbure de la trajectoire en cm*/
    double ratio;
    double a;
    double ray; /*en cm*/
	a = 13; /*espacement des roues en cm*/
	ratio = (ray-(a/2))/(ray+a/2);
    printf("Le rapport de vitesses des roues est %f\n", ratio);
	return ratio;
}


CORO_CONTEXT( manual_command );
CORO_CONTEXT( manual_drive );

CORO_CONTEXT( handle_sonar_measures );
CORO_CONTEXT( handle_gyro_measures );
CORO_CONTEXT( handle_compass_measures );
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
            tacho_set_speed_sp( MOTOR_BOTH, -speed_linear );
            tacho_run_forever( MOTOR_BOTH );
            break;
        case BACK:
            tacho_set_speed_sp( MOTOR_BOTH, speed_linear );
            tacho_run_forever( MOTOR_BOTH );
            break;
        case LEFT:
            tacho_set_speed_sp( MOTOR_LEFT, speed_circular );
            tacho_set_speed_sp( MOTOR_RIGHT, -speed_circular );
            tacho_run_forever( MOTOR_BOTH );
            break;
        case RIGHT:
            tacho_set_speed_sp( MOTOR_LEFT, -speed_circular );
            tacho_set_speed_sp( MOTOR_RIGHT, speed_circular );
            tacho_run_forever( MOTOR_BOTH );
            break;
        }
        state = command;
    }
    CORO_END();
}

CORO_DEFINE( handle_sonar_measures ) {

  CORO_BEGIN();
  if ( us == SOCKET__NONE_ ) {CORO_QUIT();}
  us_distance_old = us_distance;
  us_distance = sensor_get_value0(us, 0);
  printf("Sonar distance = %f\n", us_distance);
  CORO_YIELD();
  CORO_END();
}

CORO_DEFINE( handle_gyro_measures ) {

  CORO_BEGIN();
  if ( gyro == SOCKET__NONE_ ) {CORO_QUIT();}
  gyro_angle_old = gyro_angle;
  gyro_angle = sensor_get_value0(gyro, 0);
  printf("Gyro angle = %d\n", gyro_angle);
  CORO_YIELD();
  CORO_END();
}

CORO_DEFINE( handle_compass_measures ) {

  CORO_BEGIN();
  if ( compass == SOCKET__NONE_ ) {CORO_QUIT();}
  compass_direction_old = compass_direction;
  compass_direction = sensor_get_value0(compass, 0);
  printf("Compass direction = %d\n", compass_direction);
  compass_direction -= circuit_direction;
  printf("Compass direction after computation = %d\n", compass_direction);
  CORO_YIELD();
  CORO_END();
}

CORO_DEFINE( race_map ) {
  CORO_LOCAL int ray;
  CORO_BEGIN();
  switch(position_in_circuit){
  case MIDDLE_RIGHT:
    if (position_in_circuit != position_in_circuit_old){
      ratio = 90;
      if (compass_direction > 1  && compass_direction < 180){
        command = LEFT;
      } else if (compass_direction >= 180  && compass_direction < 359){
        command = RIGHT;
      } else {
        command = FORTH;
        position_in_circuit_old == position_in_circuit;
      }
    } else {
      if (us_distance <= us_distance_to_main_wall){
        position_in_circuit = UP_RIGHT;
        command = LEFT;
        ratio = calcul_ratio(30);
      }
    }
    break;
  case UP_RIGHT:
    if (compass_direction > 268 && compass_direction < 272){
      position_in_circuit = UP_LEFT;
    }
    break;
  case UP_LEFT:
    if (position_in_circuit != position_in_circuit_old){
      if (us_distance <= us_distance_to_main_wall){
        position_in_circuit_old = position_in_circuit;
        command = LEFT;
        ratio = calcul_ratio(30);
      }
    } else if (compass_direction > 178 && compass_direction < 182){
      position_in_circuit = MIDDLE_LEFT;
    }
    break;
  case MIDDLE_LEFT:
    if (position_in_circuit != position_in_circuit_old){
      ratio = 90
      if (compass_direction > 165  && compass_direction < 344){
        command = LEFT;
      } else if (compass_direction <= 165  || compass_direction > 346){
        command = RIGHT;
      } else {
        command = FORTH;
        position_in_circuit_old == position_in_circuit;
      }
    } else {
      if (us_distance <= us_distance_to_middle_wall) {
        ratio = 90;
        if (compass_direction > 1  && compass_direction < 180){
          command = LEFT;
        } else if (compass_direction >= 180  && compass_direction < 359){
          command = RIGHT;
        } else {
          command = FORTH;
          position_in_circuit = BOTTOM_LEFT;
        }
      }
    }
    break;
  case BOTTOM_LEFT:
    if (position_in_circuit != position_in_circuit_old){
      if (us_distance <= us_distance_to_main_wall){
        position_in_circuit_old = position_in_circuit;
        command = LEFT;
        ratio = calcul_ratio(30);
      }
    } else if (compass_direction > 88 && compass_direction < 92){
      position_in_circuit = BOTTOM_RIGHT;
    }
    break;
  case BOTTOM_RIGHT:
    if (compass_direction < 2  || compass_direction > 358){
      position_in_circuit = MIDDLE_RIGHT;
    }
    break;
  }
  CORO_END();
}

CORO_DEFINE( race_drive ) {
  CORO_LOCAL int speed_exterior, speed_interior;
  CORO_LOCAL int state = STOP;
  CORO_BEGIN();
  speed_exterior = max_speed * SPEED_LINEAR / 100;
  speed_interior = speed_exterior * ratio;
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
          tacho_set_speed_sp( MOTOR_BOTH, speed_exterior );
          tacho_run_forever( MOTOR_BOTH );
          break;
      case BACK:
          tacho_set_speed_sp( MOTOR_BOTH, -speed_exterior );
          tacho_run_forever( MOTOR_BOTH );
          break;
      case LEFT:
          tacho_set_speed_sp( MOTOR_LEFT, speed_interior );
          tacho_set_speed_sp( MOTOR_RIGHT, speed_exterior );
          tacho_run_forever( MOTOR_BOTH );
          break;
      case RIGHT:
          tacho_set_speed_sp( MOTOR_LEFT, speed_exterior );
          tacho_set_speed_sp( MOTOR_RIGHT, speed_interior );
          tacho_run_forever( MOTOR_BOTH );
          break;
      }
      state = command;
  }
  CORO_END();

}

int main( int argc, char *argv[] ) {

    int manual = 0;
    int position_in_circuit = MIDDLE_RIGHT;
    int position_in_circuit_old = MIDDLE_RIGHT;
    if (argc > 1) {
      if (argv[1] == "-help"){
        printf("Help message to be written here\n");
      } else if (argv[1] == "-manual") {
        manual = 1;
      }
    }
    printf( "Waiting the EV3 brick online...\n" );
    if ( !brick_init()) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
    alive = init();
    if (manual) {
      while ( alive ) {
          CORO_CALL( manual_command );
          CORO_CALL( manual_drive );
          sleep_ms( 10 );
      }
    } else {
      while ( alive ) {
          CORO_CALL( handle_sonar_measures );
          CORO_CALL( handle_gyro_measures );
          CORO_CALL( handle_compass_measures );
          CORO_CALL( race_map );
          CORO_CALL( race_drive );
          sleep_ms( 10 );
      }
    }
    brick_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    return ( 0 );
}
