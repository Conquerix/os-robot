#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "coroutine.h"
#include "brick.h"
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

int manual = 0;
int circuit_direction = 0;

// Sensors Addresses

POOL_T us;
POOL_T gyro;
POOL_T compass;

// Global Variables used between coroutines

float us_distance, us_distance_old, us_distance_expected;
int gyro_angle, gyro_angle_old, gyro_angle_expected;
int compass_direction, compass_direction_old, compass_direction_expected;

int compass_direction_uncomputed;

int compass_measure, compass_measure_uncomputed;
int compass_circuit;

int close_to_middle_wall = 0;

// Map global variables

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
float us_distance_to_middle_wall = 250;

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
      circuit_direction = sensor_get_value0(gyro, 0);
    } else {printf("-/!\\- Gyro Not Found !! -/!\\-\n");}

    compass = sensor_search( HT_NXT_COMPASS );
    if (compass) {
      printf("--- Compass Found !! ---\n");
      sensor_set_mode(compass, COMPASS_DIRECTION);
      compass_circuit = sensor_get_value0(compass, 0);
    } else {printf("-/!\\- Compass Not Found !! -/!\\-\n");}

    if (!(us && gyro)) {
      printf("-/!\\- Some sensors were not found -/!\\-\n");
      printf("--- Please use the brick buttons ---\n");
      manual = 1;
    }

    printf( "Press BACK on the EV3 brick for EXIT...\n" );
    return ( 1 );
}

int calcul_ratio(double ray){
  /* prend en argument le rayon de courbure de la trajectoire en cm*/
  double ratio;
  double a;
	a = 13; /*espacement des roues en cm*/
	ratio = (ray-(a/2))/(ray+a/2) * 100;
  printf("Le rapport de vitesses des roues est %f\n", ratio);
	return ratio;
}

void map_middle_right(){
  if (position_in_circuit != position_in_circuit_old){
    ratio = 50;
    if (compass_direction < 179){
      command = LEFT;
    } else if (compass_direction > 181){
      command = RIGHT;
    } else {
      command = FORTH;
      position_in_circuit_old = position_in_circuit;
    }
  } else {
    if (us_distance <= 150){
      position_in_circuit = UP_RIGHT;
      //circuit_direction = compass_direction_uncomputed; //Compass calibration
      command = LEFT;
      ratio = 50; //calcul_ratio(30);
      printf("Ratio = %d\n", ratio);
    }
  }
  return;
}

void map_up_right(){
  if (compass_direction >= 90 && compass_direction <= 94){
    position_in_circuit = UP_LEFT;
    circuit_direction = compass_direction_uncomputed;
    //circuit_direction = (compass_direction_uncomputed + 90) % 360; //Compass calibration
    command = FORTH;
  }
  return;
}

void map_up_left(){
  if (position_in_circuit != position_in_circuit_old){
    if (us_distance <= 150){
      position_in_circuit_old = position_in_circuit;
      command = LEFT;
      ratio = 50;//calcul_ratio(30);
    }
  } else if (compass_direction >= 90 && compass_direction <= 94){
    position_in_circuit = MIDDLE_LEFT;
    circuit_direction = compass_direction_uncomputed;
    //circuit_direction = (compass_direction_uncomputed + 180) % 360; //Compass calibration
  }
  return;
}

void map_middle_left(){
  if (us_distance <= us_distance_to_middle_wall) {
    close_to_middle_wall = 1;
  }
  if (position_in_circuit != position_in_circuit_old){
    ratio = 50;
    if (compass_direction < 169){
      command = RIGHT;
    } else if (compass_direction > 171){
      command = LEFT;
    } else {
      command = FORTH;
      position_in_circuit_old = position_in_circuit;
    }
  } else if (close_to_middle_wall){
    ratio = 50;
    if (compass_direction < 179){
      command = RIGHT;
    } else if (compass_direction > 181){
      command = LEFT;
    } else {
      command = FORTH;
      position_in_circuit = BOTTOM_LEFT;
      circuit_direction = compass_direction_uncomputed;
      //circuit_direction = (compass_direction_uncomputed + 180) % 360; //Compass calibration
      close_to_middle_wall = 0;
    }
  }
  return;
}

void map_bottom_left(){
  if (position_in_circuit != position_in_circuit_old){
    if (us_distance <= 150){
      position_in_circuit_old = position_in_circuit;
      command = LEFT;
      ratio = 50; //calcul_ratio(30);
    }
  } else if (compass_direction >= 90 && compass_direction <= 94){
    position_in_circuit = BOTTOM_RIGHT;
    circuit_direction = compass_direction_uncomputed;
    command = FORTH;
    //circuit_direction = (compass_direction_uncomputed + 270) % 360; //Compass calibration
  }
  return;
}

void map_bottom_right(){
  if (position_in_circuit != position_in_circuit_old){
    if (us_distance <= 300){
      position_in_circuit_old = position_in_circuit;
      command = LEFT;
      ratio = 50;//calcul_ratio(30);
    }
  } else if (compass_direction >= 90 && compass_direction <= 94){
    circuit_direction = compass_direction_uncomputed;
    position_in_circuit = MIDDLE_RIGHT;
    //circuit_direction = (compass_direction_uncomputed + 180) % 360; //Compass calibration
  }
  return;
}

CORO_CONTEXT( manual_command );
CORO_CONTEXT( manual_drive );

CORO_CONTEXT( sonar_command );
CORO_CONTEXT( sonar_drive );

CORO_CONTEXT( race_stop );
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

CORO_DEFINE( sonar_command ) {
    CORO_LOCAL uint8_t keys, pressed = EV3_KEY__NONE_;
    CORO_BEGIN();
    printf("Test 1");
    for ( ; ; ) {
        /* Waiting any brick's key is pressed or released */
        CORO_WAIT(( keys = brick_keys()) != pressed );
        pressed = keys;
        switch ( pressed ) {
        /* Quit */
        case EV3_KEY_BACK:
            command = STOP;
            alive = 0;
            printf("Test 2");
            break;
        /* Stop */
        case EV3_KEY_UP:
        case EV3_KEY_DOWN:
        case EV3_KEY_CENTER:
            command = STOP;
            printf("Test 3");
            break;
        /* Left */
        case EV3_KEY_RIGHT:
            command = LEFT;
            printf("Test 4");
            break;
        /* Right */
        case EV3_KEY_LEFT:
            command = RIGHT;
            printf("Test 5");
            break;
        }
        CORO_YIELD();
    }
    CORO_END();
}

CORO_DEFINE( sonar_drive ) {
    CORO_LOCAL int state = STOP;
    CORO_LOCAL int sonar_pos = 0;
    CORO_BEGIN();
    printf("Test A");
    tacho_set_speed_sp( MOTOR_SONAR, max_speed * 5 / 100 );
    for ( ; ; ) {
        /* Waiting new command */
        CORO_WAIT( command != state );
        sonar_pos = tacho_get_position_sp(MOTOR_SONAR, 0);
        switch ( command ) {
        case STOP:
            tacho_set_position_sp(MOTOR_SONAR, 0);
            tacho_run_to_abs_pos(MOTOR_SONAR);
            printf("Test B");
            break;
        case LEFT:
            tacho_set_position_sp( MOTOR_SONAR, 20 );
            tacho_run_to_abs_pos(MOTOR_SONAR);
            printf("Test D");
            break;
        case RIGHT:
            tacho_set_position_sp( MOTOR_SONAR, -20 );
            tacho_run_to_abs_pos(MOTOR_SONAR);
            printf("Test E");
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
        if (pressed == EV3_KEY_BACK){
            command = STOP;
            alive = 0;
        }
        CORO_YIELD();
    }
    CORO_END();
}

CORO_DEFINE( handle_sonar_measures ) {

  if ( us == SOCKET__NONE_ ) {CORO_QUIT();}
  for(;;){
    us_distance_old = us_distance;
    us_distance = sensor_get_value0(us, 0);
    printf("Sonar distance = %f\n", us_distance);
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( handle_gyro_measures ) {

  CORO_BEGIN();
  if ( gyro == SOCKET__NONE_ ) {CORO_QUIT();}
  for (;;){
    compass_direction_old = compass_direction;
    compass_direction_uncomputed = sensor_get_value0(gyro, 0);
    printf("Gyro angle = %d\n", compass_direction_uncomputed);
    compass_direction = compass_direction_uncomputed - circuit_direction;
    compass_direction += 180;
    compass_direction = compass_direction % 360;
    if (compass_direction < 0) {compass_direction += 360;}
    printf("Compass direction after computation = %d\n", compass_direction);
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( handle_compass_measures ) {

  CORO_BEGIN();
  if ( compass == SOCKET__NONE_ ) {CORO_QUIT();}
  for (;;){
    /* printf("Test4.2 - ");
    compass_direction_old = compass_direction;
    compass_direction_uncomputed = sensor_get_value0(compass, 0);
    printf("Compass direction = %d - ", compass_direction);
    compass_direction = compass_direction_uncomputed - circuit_direction;
    compass_direction += 360;
    compass_direction = compass_direction % 360;
    printf("Compass direction after computation = %d\n", compass_direction); */
    compass_measure_uncomputed = sensor_get_value0(compass, 0);
    compass_measure = compass_measure_uncomputed - compass_circuit + 360;
    compass_measure = compass_measure % 360;
    printf("Compass measure = %d\n", compass_measure);
    CORO_YIELD();
  }
  CORO_END();
}

CORO_DEFINE( race_map ) {
  CORO_LOCAL int ray;
  CORO_BEGIN();
  for(;;){
    switch(position_in_circuit){
    case MIDDLE_RIGHT:
      printf("MIDDLE RIGHT \n");
      wheel_speed = 100;
      map_middle_right();
      break;
    case UP_RIGHT:
      printf("UP_RIGHT\n");
      wheel_speed = 40;
      map_up_right();
      break;
    case UP_LEFT:
      printf("UP_LEFT\n");
      wheel_speed = 40;
      map_up_left();
      break;
    case MIDDLE_LEFT:
      printf("MIDDLE_LEFT");
      wheel_speed = 100;
      map_middle_left();
      break;
    case BOTTOM_LEFT:
      printf("BOTTOM_LEFT\n");
      wheel_speed = 40;
      map_bottom_left();
      break;
    case BOTTOM_RIGHT:
      printf("BOTTOM_RIGHT");
      wheel_speed = 40;
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
  speed_exterior = max_speed * wheel_speed / 100;
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
          printf("Command : FORTH\n");
          break;
      case BACK:
          printf("Test6.4 - ");
          tacho_set_speed_sp( MOTOR_BOTH, -speed_exterior );
          tacho_run_forever( MOTOR_BOTH );
          printf("Command : BACK\n");
          break;
      case LEFT:
          printf("Test6.5 - ");
          tacho_set_speed_sp( MOTOR_LEFT, speed_interior );
          tacho_set_speed_sp( MOTOR_RIGHT, speed_exterior );
          tacho_run_forever( MOTOR_BOTH );
          printf("Command : LEFT\n");
          break;
      case RIGHT:
          printf("Test6.6\n");
          tacho_set_speed_sp( MOTOR_LEFT, speed_exterior );
          tacho_set_speed_sp( MOTOR_RIGHT, speed_interior );
          tacho_run_forever( MOTOR_BOTH );
          printf("Command : RIGHT\n");
          break;
      }
      state = command;
      ratio_state = ratio;
  }
  CORO_END();

}

int main( int argc, char *argv[] ) {

    int manual = 0;
    int manual_sonar = 0;
    if (argc > 1) {
      if (argv[1][0] == '1'){
        printf("Help message to be written here\n");
        return ( 0 );
      } else if (argv[1][0] == '2') {
        manual = 1;
      } else if (argv[1][0] == '3') {
        manual_sonar = 1;
        printf("Sonar control mode\n");
      }
    }
    printf( "Waiting the EV3 brick online...\n" );
    if ( !brick_init()) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
    alive = init();
    if (manual) {
      while ( alive ) {
        CORO_CALL( handle_sonar_measures );
        CORO_CALL( handle_gyro_measures );
        CORO_CALL( handle_compass_measures );
        CORO_CALL( manual_command );
        CORO_CALL( manual_drive );
        sleep_ms( 10 );
      }
    } else if (manual_sonar) {
      while ( alive ) {
        //CORO_CALL( handle_sonar_measures );
        //CORO_CALL( handle_gyro_measures );
        //CORO_CALL( handle_compass_measures );
        CORO_CALL( sonar_command );
        CORO_CALL( sonar_drive );
        sleep_ms( 10 );
      }
    } else {
      while ( alive ) {
        CORO_CALL( race_stop );
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
