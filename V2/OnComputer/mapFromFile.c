#include "utils.h"

int mapFromFile(FILE *file, int map[HAUTEUR_GRILLE][LARGEUR_GRILLE], Position *robot_pos, intPoint *obj_cell) {
	char c = '\0';
	int h, w;

	rewind(file);
	for(h = 0; h < size.height; h++) {
		for(w = 0; (c = fgetc(file)) && w < size.width && !feof(file); w++) {
			// Detects if the grid is incomplete, prints location and returns 1 (error)
			if(c == '\n' || (c != '#' && c != '/' && c != ' ' && c != 'S' && c != 'G')) {
				fprintf(stderr,"Error: Grid data is incomplete at x: %d, y: %d\n", w + 1, h + 1);
				return 1;
			}
			// Saves file character to grid
			switch(c){
        case '#':
        map[h][w] = 3;
        break;

        case '/':
        map[h][w] = 2;
        break;

        case ' ':
        map[h][w] = 0;
        break;

				case 'S':
				map[h][w] = 0;
				(*robot_pos).x = (2*h + 1)/2;
				(*robot_pos).y = (2*w + 1)/2;
				(*robot_pos).direction = 0;
        break;

				case 'G':
				map[h][w] = 0;
				(*obj_cell).x = h;
				(*obj_cell).y = w;
        break;
      }
		}
	}
	return 0;
}
