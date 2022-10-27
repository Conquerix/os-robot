#include "utils.h"

//We give an id to each cell of the grid to access them more easily. These 2 functions allow us to get the cell from its id and vice versa.
intPoint cellIdConversion(int cell_id){
  intPoint cell_pos;
  cell_pos.x = cell_id / LARGEUR_GRILLE;
  cell_pos.y = cell_id % LARGEUR_GRILLE;
  return cell_pos;
}

int idCellConversion(intPoint cell){
  int cell_id;
  cell_id = cell.x*LARGEUR_GRILLE + cell.y;
  return cell_id;
}
