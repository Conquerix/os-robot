#include "utils.h"

motorCommand robotToPointV1(Position robot_position, Point destination){
  float x_from_robot, y_from_robot;
  motorCommand command;
  fprintf(stderr, "Destination (%f - %f)\n", destination.x, destination.y);
  x_from_robot = destination.x - robot_position.x;
  y_from_robot = destination.y - robot_position.y;
  fprintf(stderr, "From Robot : (%f - %f)\n", x_from_robot, y_from_robot);
  float angle = robot_position.direction - (asinf(x_from_robot/sqrt(x_from_robot * x_from_robot + y_from_robot * y_from_robot)) * 180 / PI);

  if (angle < 0) {
    while (angle < 0){
      angle += 360;
    }
  } else {
    while (angle >= 360) {
      angle -= 360;
    }
  }
  if (aroundEqual(angle, 0.0, 1)){
    command.left = 100;
    command.right = 100;
  } else if (angle <= 180){
    command.left = -30;
    command.right = 30;
  } else {
    command.left = 30;
    command.right = -30;
  }
  if (aroundEqual(x_from_robot+y_from_robot, 0.0, 1)){
    command.left = 0;
    command.right = 0;
  }
  return command;
}
