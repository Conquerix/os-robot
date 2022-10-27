/* path.c
   This program reads maze data from a file, solves
   the shortest path, and outputs the solution to a new file.
   Currently the algorithm uses four directions (90 degrees)
   but can be easily modified to support any angles since
   the node generation is based upon sin and cos functions
   by Devon Crawford (dcrawf04@mail.uoguelph.ca)
   0973673
   Dec 2. 2016
*/

// Main function removed as we adapted the algorithm to our program.

#include "utils.h"

// Returns node with lowest F value
Node lowestF(Node open[][size.width], int length) {
	// Copies open nodes to new 1D array for sorting
	Node sort[length];
	arrangeID(sort, open, length);

	int Switch = -1;
	Node curr, next;
	int i;

	// Sorts open array from low to high
	while(Switch != 0) {
		Switch = 0;

		// Iterates each existing open node (length)
		for(i = 0; i < length - 1; i++) {
			curr = sort[i];
			next = sort[i + 1];

			// SWAP: if curr Node has greater f than next Node
			if(curr.f > next.f) {
				sort[i] = next;
				sort[i + 1] = curr;
				Switch = 1;
			}
		}
	}
	// Returns node with lowest f
	return sort[0];
}

// Main recursive function to find open nodes, new parent, and repeat
void findPath(char grid[][size.width], Node open[][size.width], Node closed[][size.width], Node path[][size.width], Node start, Node goal, Node parent, int* openLength, int* closedLength, int* pathLength) {
	int i;
	// Checks all 8 directions (diagonals too)
	for(i = 0; i < 8; i++) {

		// Uses cosine and sine functions to get circle of points (at 45 degree intervals) around parent
		int possibleX = parent.x + round(-1 * cos(PI_4 * i));
		int possibleY = parent.y + round(-1 * sin(PI_4 * i));

		// If coordinates are outside of boundaries
		if(possibleX < 0 || possibleY < 0 || possibleX > (size.width - 1) || possibleY > (size.height - 1)) {
			continue;
		}

		// Does not make a new open node IF
		// Node is a border, node is already in the open list
		// Or the node is already in the closed list
		if(grid[possibleY][possibleX] == '#' || closed[possibleY][possibleX].exists == 1 || open[possibleY][possibleX].exists == 1) {
			continue;
		}
		// Create an open node, calculate the values and add it to the open list
		calculateNodeValues(parent, goal, possibleX, possibleY, grid, open, openLength);
	}
	// Sets new parent as openNode with the lowest F cost
	parent = lowestF(open, *openLength);
	// If chosen parent does not exist..
	// Must be no path, exhausted entire map
	if(parent.exists != 1) {
		// Inform user that no path was found
		printf("\n!! No path exists !!\n");

		// Breaks out of entire pathfinding functions
		return;
	}
	// If parent node is the goal!
	else if(grid[parent.y][parent.x] == 'G') {
		// Get the last node in the closed list
		Node lastClosed = getNode(closed, (*closedLength)-1);

		// Connect path by tracing back parents, starting at lastClosed Node
		connectPath(path, pathLength, lastClosed);
		return;
	}
	// Remove parent from open list
	removeListNode(open, openLength, parent);
	// Adds parent to closed list
	addListNode(closed, closedLength, parent);
	// Recursive call to function, replacing parent as the open node with lowest fCost
	findPath(grid, open, closed, path, start, goal, parent, openLength, closedLength, pathLength);
}

// Parent pointers are followed from goal until it reaches start
void connectPath(Node path[][size.width], int *pathLength, Node parent) {
	while(parent.parent != NULL) {
		// Adds parent to the path list
		addListNode(path, pathLength, parent);
		// Parent Node is overwritten by parent Node of itself
		parent = *(parent.parent);
	}
	// Flips id's in path because they were recorded in reverse
	reverseID(path, *pathLength);
}
