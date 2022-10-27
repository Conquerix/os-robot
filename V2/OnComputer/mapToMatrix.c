int * mapToMatrix(int map[HAUTEUR_GRILLE][LARGEUR_GRILLE]){

  static int graph[V][V];
  int i,j;

  for (i=0;i<V;i++){
    for (j=0;j<V;j++){
      if (i==j){
        graph[i][j] = 0;
      } else {
        graph[i][j] = 5;
      }
    }
  }

  for (i=0;i<HAUTEUR_GRILLE-1;i++){
    for (j=0;j<LARGEUR_GRILLE-1;j++){
      if (map[i][j] == 0){
        if (map[i+1][j] == 0){
          graph[i*LARGEUR_GRILLE + j][(i+1)*LARGEUR_GRILLE + j] = 1;
          graph[(i+1)*LARGEUR_GRILLE + j][i*LARGEUR_GRILLE + j] = 1;
        }
        if (map[i][j+1] == 0){
          graph[i*LARGEUR_GRILLE + j][i*LARGEUR_GRILLE + (j+1)] = 1;
          graph[i*LARGEUR_GRILLE + (j+1)][i*LARGEUR_GRILLE + j] = 1;
        }
      }
    }
    if (map[i][LARGEUR_GRILLE-1] == 0){
      if (map[i+1][LARGEUR_GRILLE-1] == 0){
        graph[i*LARGEUR_GRILLE + LARGEUR_GRILLE-1][(i+1)*LARGEUR_GRILLE + LARGEUR_GRILLE-1] = 1;
        graph[(i+1)*LARGEUR_GRILLE + LARGEUR_GRILLE-1][i*LARGEUR_GRILLE + LARGEUR_GRILLE-1] = 1;
      }
    }
  }

  for (j=0;j<LARGEUR_GRILLE-1;j++){
    if (map[HAUTEUR_GRILLE-1][j] == 0){
      if (map[HAUTEUR_GRILLE-1][j+1] == 0){
        graph[(HAUTEUR_GRILLE-1)*LARGEUR_GRILLE + j][(HAUTEUR_GRILLE-1)*LARGEUR_GRILLE + (j+1)] = 1;
        graph[(HAUTEUR_GRILLE-1)*LARGEUR_GRILLE + (j+1)][(HAUTEUR_GRILLE-1)*LARGEUR_GRILLE + j] = 1;
      }
    }
  }

  for(i=0;i<V;i++){
    for(j=0;j<V;j++){
      if(graph[i][j] == 5){
        printf("-/!\\- ERROR WHEN CONVERSION FROM MAP TO GRAPH -/!\\-\n");
      }
    }
  }

  return graph;
}
