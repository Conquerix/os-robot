#include "utils.h"

int circle(intPoint cell, Point sonar_position, Point object_position){
    float x;
    float y;

    float x1 = sonar_position.x;
    float y1 = sonar_position.y;
    float x2 = object_position.x;
    float y2 = object_position.y;

    Point cellPos = cellToPos(cell);

    /* équation de la droite:*/

    float a = (y2-y1)/(x2-x1);
    float b = y2 - a*x2;

    /* construire les points qui formeront la droite reliant l'objet au sonar */

    Point segment[10000];
    int whereInSegment = 0;

    if(x1<x2){
        x = x1;
    } else {
        x = x2;
    }
    Point tmpPoint;
    while (fabs(x-x1) <= fabs(y2-y1)) {
        y = a*x + b;
        if (fabs(y-y1) <= fabs(y2-y1)) {
            x = x + 0.5;
            tmpPoint.x = x;
            tmpPoint.y = y;
            segment[whereInSegment] = tmpPoint;
            whereInSegment++;
        } else {
            break;
        }
    }

    /* return True si la droite passe dans un cercle de rayon celui de la case */
    int i;
    for (i=0; i < whereInSegment+1; i++) {

        tmpPoint.x = segment[i].x;
        tmpPoint.y = segment[i].y;

        if ((x - cellPos.x)*(x - cellPos.x) + (y - cellPos.y)*(y - cellPos.y) <= 1) {
                return true;
        }
    }
    return false;
}


void updateMap(Point object_position, Position sonar_position, int map[HAUTEUR_GRILLE][LARGEUR_GRILLE]){
    int i;
    int j;

    float xref;
    float yref;

    Point tmpPoint;
    intPoint tmpIntPoint;

    tmpPoint.x = sonar_position.x;
    tmpPoint.y = sonar_position.y;

    intPoint pos1 = posToCell(tmpPoint);
    intPoint pos2 = posToCell(object_position);

    float x = fabs(pos2.x - pos1.x);
    float y = fabs(pos2.y - pos1.y);

    if (pos2.x > pos1.x) {
      xref = pos1.x;
    } else {
      xref = pos2.x;
    }

    if (pos2.y > pos1.y) {
      yref = pos1.y;
    } else {
      yref = pos2.y;
    }

    /* parcourir l'ensemble des cases à l'intérieur du rectange de largeur Y et de hauteur X et les rendre libre si besoin */
    for (i = 1; i < x; i++){
        for (j = 1; j < y; j++){
            tmpIntPoint.x = (int)round(xref+i);
            tmpIntPoint.y = (int)round(yref+j);
            if (circle(tmpIntPoint, tmpPoint, object_position)) {
                if (map[tmpIntPoint.x][tmpIntPoint.y] == 1){
                    map[tmpIntPoint.x][tmpIntPoint.y] = 0;
                } /*else if (map[X1+i;Y1+j] == 3) {
                    lost_function();
                }*/
            }
        }
    }
    if (map[(int)round(xref+x)][(int)round(yref+y)] == 0) {
      map[(int)round(xref+x)][(int)round(yref+y)] = 1;
    }
}




/** to check if maps are equal **/


bool mapsAreEqual( int map1[HAUTEUR_GRILLE][LARGEUR_GRILLE], int map2[HAUTEUR_GRILLE][LARGEUR_GRILLE]) {

	int i = 0;
	int j = 0;
	bool b = true;

	/* j'imagine que largeur et hauteur sont des variables globales ? */

	while (b && i < HAUTEUR_GRILLE){
    while (b && j < LARGEUR_GRILLE){
			if (map1[i][j] != map2[i][j]) {
				b = false;
      }
      j++;
		}
    i++;
	}

	return b;
}



Point getWatchedObject(Position sonar_position, int sonar_distance) {

	float x;
	float y;
	Point object_position;

	int dir = sonar_position.direction * 2 * PI / 360;

	y = sinf(dir) * sonar_distance;
	x = cosf(dir) * sonar_distance;

	object_position.x = sonar_position.y - y;
	object_position.y = sonar_position.x + x;

	return object_position;}
