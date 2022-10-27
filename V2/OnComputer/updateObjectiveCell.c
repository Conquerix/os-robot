#include "utils.h"


//we use this function to define the new starting point and goal corresponding to the actual position of the robot

void updateObjectiveCell(intPoint objective_cell, Position robot_position){

  if (objective_cell.x == 25 && objective_cell.y == 30){
    if (robot_position.x < 90){
      objective_cell.x = 50;
      objective_cell.y = 15;
    }
  } else if (objective_cell.x == 50 && objective_cell.y == 15){
    if (robot_position.y < 50){
      objective_cell.x = 75;
      objective_cell.y = 30;
    }
  } else if (objective_cell.x == 75 && objective_cell.y == 30){
    if (robot_position.x > 110){
      objective_cell.x = 50;
      objective_cell.y = 45;
    }
  } else if (objective_cell.x == 50 && objective_cell.y == 45){
    if (robot_position.y > 70){
      objective_cell.x = 25;
      objective_cell.y = 30;
    }
  }
}
