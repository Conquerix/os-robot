#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>

#define PI   3.14159
#define PI_2 3.14159/2
#define PI_4 3.14159/4

#define HAUTEUR_CIRCUIT 200.0
#define LARGEUR_CIRCUIT 120.0
#define HAUTEUR_GRILLE 100
#define LARGEUR_GRILLE 60
#define V (HAUTEUR_GRILLE * LARGEUR_GRILLE)

typedef struct motorCommand{
  int left;
  int right;
} motorCommand;

typedef struct Point{
  float x;
  float y;
} Point;

typedef struct intPoint{
  int x;
  int y;
} intPoint;

typedef struct Position{
  float x;
  float y;
  int direction;
} Position;

// Creating GridSize struct
typedef struct GridSize {
	int width;
	int height;
} GridSize;

// Creating Node struct
typedef struct Node {
	struct Node* parent;
	int exists;
	int x;
	int y;
	int g;
	int h;
	int f;
	int id;
} Node;

GridSize size;




#endif
