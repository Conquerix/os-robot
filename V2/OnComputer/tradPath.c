#include "utils.h"

/** Path (calculated by the algo) traduction in a dynamic array **/

void pathToPathList(Node pathGrid[][size.width], intPoint * list, int whereInPath, int pathLength, Node start, Node goal) {


	/* conversion */
	intPoint start2;
	start2.x = start.y;
	start2.y = start.x;

	/* append to list */
	list[whereInPath] = start2;

	/* check if we are at the end or not */
	if((start.x != goal.x || start.y != goal.y) && whereInPath < pathLength -1){
		int i = 0;
		for(i = 0; i < 8; i++) {
			// Uses cosine and sine functions to get circle of points (at 45 degree intervals) around parent
			int possibleX = start.x + round(-1 * cos(PI_4 * i));
			int possibleY = start.y + round(-1 * sin(PI_4 * i));

			// If coordinates are outside of boundaries break the loop
			if(possibleX < 0 || possibleY < 0 || possibleX > (size.width - 1) || possibleY > (size.height - 1)) {
				continue;
			}


			if(pathGrid[possibleY][possibleX].exists == 1) {
				pathGrid[start.y][start.x].exists = 0;
				start = pathGrid[possibleY][possibleX];
				pathToPathList(pathGrid, list, whereInPath+1, pathLength, start, goal);
			}
		}
	}
}
