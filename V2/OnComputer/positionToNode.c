#include "utils.h"

Node positionToNode(Position pos){
  Node node;
  Point posPoint;
  intPoint cell;

  posPoint.x = pos.x;
  posPoint.y = pos.y;

  cell = posToCell(posPoint);

  // We swich x and y because of different implementations.
  node.x = cell.y;
  node.y = cell.x;
  node.exists = 1;

  return node;
}
