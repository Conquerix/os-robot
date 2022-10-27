#include "utils.h"

intPoint posToCell(Point pos){
  intPoint int_pos;
  float cell_side_length = HAUTEUR_CIRCUIT / HAUTEUR_GRILLE;
  int i;
  for (i=0;i<HAUTEUR_CIRCUIT;i++){
    if (pos.x >= i*cell_side_length){
      if (pos.x < (i+1)*cell_side_length){
        int_pos.x = i;
      }
    }
  }
  for (i=0;i<LARGEUR_CIRCUIT;i++){
    if (pos.y >= i*cell_side_length){
      if (pos.y < (i+1)*cell_side_length){
        int_pos.y = i;
      }
    }
  }
  return int_pos;
}

Point cellToPos(intPoint int_pos){
  Point pos;
  //float cell_side_length = HAUTEUR_CIRCUIT / HAUTEUR_GRILLE;
  pos.x = (2*int_pos.x + 1)/2;
  pos.y = (2*int_pos.y + 1)/2;
  return pos;
}
