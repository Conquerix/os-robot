#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "coroutine.h"
#include "brick.h"

#define MOTOR_LEFT     OUTD
#define MOTOR_RIGHT    OUTA
#define MOTOR_BOTH     ( MOTOR_LEFT | MOTOR_RIGHT )

#define MOTOR_SONAR    OUTC

int init( void ) {
    if ( tacho_is_plugged( MOTOR_BOTH, TACHO_TYPE__NONE_ )) {  /* any type of motor */
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

    printf( "Press BACK on the EV3 brick for EXIT...\n" );
    return ( 1 );
}

int main(){
  char * buffer;
  buffer = malloc(sizeof(char)*1000);
  buffer = tacho_get_commands(MOTOR_LEFT, buffer, 1000);
  printf("Motor left : %s\n", buffer);
  buffer = tacho_get_commands(MOTOR_RIGHT, buffer, 1000);
  printf("Motor right : %s\n", buffer);
  buffer = tacho_get_commands(MOTOR_SONAR, buffer, 1000);
  printf("Motor sonar : %s\n", buffer);

  return 0;
}
