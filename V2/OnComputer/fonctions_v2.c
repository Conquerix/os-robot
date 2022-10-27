#include "utils.h"

int getWheelAngleDifference(int wheel_pos_old, int wheel_pos){
// int_wheel_pos_old between -32768 and 32767

int wheel_angle_mov; //relative integer

wheel_angle_mov = wheel_pos - wheel_pos_old;

return wheel_angle_mov; }


double calcul_ratio(double ray) {
    /* prend en argument le rayon de courbure de la trajectoire en cm*/
    double ratio;
    double a;
	a = 13; /*espacement des roues en cm*/
	ratio = (ray-(a/2))/(ray+a/2);
    printf("Le rapport de vitesses des roues est %f\n", ratio);
	return ratio;
}

/*
double inverse_ratio(float left_wheel_angle_mov, float right_wheel_angle_mov) {
    //returns the ray of the trajectory of the robot
    double ratio;
    double a; // distance between the centers of the wheels
    double pinf=INFINITY;
    a = 13;
    double ray; //ray of the trajectory of the robot
    if (right_wheel_angle_mov == 0) {
        if (left_wheel_angle_mov != 0){
            //the robot turns on himself/ without moving forward or backward
            ray = 0;
        }
    }
    else {
        ratio = fabs(left_wheel_angle_mov/right_wheel_angle_mov); //positive if the robot turns clockwise
        if (0 < ratio && ratio < 1) {
            //the robot turned left
            ray = (a/2)*(1+ratio/(ratio-1)); }

        else if ( 1 < ratio ) {
            //the robot turned right
            ray = (a/2)*(1+ratio/(ratio-1)); }

        else if (ratio == 1) {
            //the robot moves in a straight line
            ray = pinf;
            }

        else if (aroundEqual(ratio, -1, 0.1)){
            //the robot is turning right on himself
            ray = 0;
        }
    }

    return ray;

}
*/

Position getNewRobotPos(Position robot_pos, float left_wheel_angle_mov, float right_wheel_angle_mov) {

    float right_wheel_distance; //distance traveled by the right wheel
    float left_wheel_distance; //distance traveled by the left wheel
    float wheel_ray; // ray of the wheel
    wheel_ray = 2.25 ; //in cm
    float wheel_distance; //distance between the wheels
    wheel_distance = 13; //in cm

    Position new_robot_pos;

    float rad_direction = (-1) * robot_pos.direction * PI / 180;
    float new_rad_direction;

    right_wheel_distance = right_wheel_angle_mov*PI*wheel_ray/180;
    left_wheel_distance  =  left_wheel_angle_mov*PI*wheel_ray/180;

    if (aroundEqual(left_wheel_distance, right_wheel_distance, 0.01)) { // basically going straight
      new_robot_pos.x = robot_pos.x + left_wheel_distance * cos(rad_direction);
      new_robot_pos.y = robot_pos.y + right_wheel_distance * sin(rad_direction);
      new_robot_pos.direction = robot_pos.direction;
    } else {
      float R = wheel_distance * (left_wheel_distance + right_wheel_distance) / (2 * (right_wheel_distance - left_wheel_distance));
      float wd = (right_wheel_distance - left_wheel_distance) / wheel_distance;

      new_robot_pos.x = robot_pos.x + R * sin(wd + rad_direction) - R * sin(rad_direction);
      new_robot_pos.y = robot_pos.y - R * cos(wd + rad_direction) + R * cos(rad_direction);
      new_rad_direction = rad_direction + wd;
      new_robot_pos.direction = (int) ((-1) * new_rad_direction * 180 / PI);
      new_robot_pos.direction = new_robot_pos.direction % 360;
      if (new_robot_pos.direction < 0) new_robot_pos.direction += 360;
    }
    return new_robot_pos;
  }

Position getSonarPosition(Position robot_pos, int sonar_direction ) {
    float d_wheel_sonar; //distance between the sonar and the wheels
    d_wheel_sonar = 18;

    //the sonar direction is relative to the robot

    Position sonar_position;

    if (0 < robot_pos.direction && robot_pos.direction < 90){
        sonar_position.x = robot_pos.x + d_wheel_sonar*sinf(robot_pos.direction*PI/180);
        sonar_position.y = robot_pos.y - d_wheel_sonar*cosf(robot_pos.direction*PI/180);
        sonar_position.direction = robot_pos.direction + sonar_direction;
    }

    else if (90 < robot_pos.direction && robot_pos.direction < 180){
        sonar_position.x = robot_pos.x + d_wheel_sonar*cosf( (robot_pos.direction - 90)*PI/180);
        sonar_position.y = robot_pos.y + d_wheel_sonar*sinf((robot_pos.direction - 90)*PI/180);
        sonar_position.direction = robot_pos.direction + sonar_direction;

    }

    else if ( 180 < robot_pos.direction && robot_pos.direction < 270 ){
        sonar_position.x = robot_pos.x - d_wheel_sonar*cosf((270 - robot_pos.direction)*PI/180);
        sonar_position.y = robot_pos.y + d_wheel_sonar*sinf((270 - robot_pos.direction)*PI/180);
        sonar_position.direction = robot_pos.direction + sonar_direction;

    }
    else if (270 < robot_pos.direction && robot_pos.direction < 360){
        sonar_position.x = robot_pos.x - d_wheel_sonar*sinf((360-robot_pos.direction)*PI/180);
        sonar_position.y = robot_pos.y - d_wheel_sonar*cosf((360-robot_pos.direction)*PI/180);
        sonar_position.direction = robot_pos.direction + sonar_direction;

    }

return sonar_position;

}
