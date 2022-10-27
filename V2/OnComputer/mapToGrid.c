#include "utils.h"

//This procedure uses the map, an array containing integers to a char grid that will be used in the astar function

void mapToGrid(int map[HAUTEUR_GRILLE][LARGEUR_GRILLE], char grid[HAUTEUR_GRILLE][LARGEUR_GRILLE]){

  int i,j;

  for (i = 0 ; i < HAUTEUR_GRILLE; i++ ) {
    for (j = 0 ; j < LARGEUR_GRILLE; j++){
        //we go through the map and change the value of the grid
      if (map[i][j] == 0){
          grid[i][j] = 'O';
      }
      else{
          grid[i][j] = '#';
      }
    }
  }
}
