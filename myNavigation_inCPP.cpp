// #include <Sparki.h>
#include <stdlib.h>     /* abs */
#include <iostream>

#define INFINITY 99         // must be larger than number_of_nodes
#define FINISHPOSITION 9    // should be navigable space

/*
 * !!!!! Remember !!!!! We have some very specifically defined things
 * Grid: In our case, a 4x4 grid. This changes with "mapSize" that will be the length of our sides on our square grid.
 * 
 * dijkstrasSolvedArray: Contains the Space Number to move to to reach goal position most quickly
 * Every index will be initialized to -1;
 * Space Number:          [1, 2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16]
 * dijkstrasSolvedArray:  [2, 3,  7,  3,  X,  X,  11, 7,  9,  9, 10, 11,  9, 10,  X, 12]
 *
 *
 * Space Number : Numbers 1,2,3,...,16. These correspond with each space on our grid
 * index :        Numbers 0,1,2,...,15. This is the index in our dijkstrasSolvedArray
 *
 *
 * Our Grid:
 * 1 is navigable, 0 is obstacle
 * Grid[i][j]:                Space Numbers
 *    0 1 2 3  =j       0   1   2   3  =j       
 *    
 * 0  1 1 1 1       0   1   2   3   4
 * 1  0 0 1 1       1   5   6   7   8
 * 2  1 1 1 1       2   9   10  11  12
 * 3  1 1 0 1       3   13  14  15  16
 * =                =
 * i                i
 *  
 * 
 * Solved dijkstra's should be: (from spaceNumber 1 to 9)
 * 
 * 2    3      7    3
 * 99   99     11   7
 * 9    9      10   11
 * 9    13(10) 99   12
 */




/* __________ Basic Setup and Global Declarations __________*/

// Times, son.
float startTime, endTime, loopTime;

// Map right hurrrr
const int mapSize = 4;
int grid[mapSize][mapSize];


/* __________ Dijkstra's Arrays __________*/
const int number_of_nodes = mapSize * mapSize;
int dijkstrasSolvedArray[number_of_nodes];
// int visitedFlag[number_of_nodes]; // 0 if unvisitied, 1 if visited


// Number to Coordinate Return
// [i,j]
int spaceNumber_to_coord_Return[2];

// currentPostion
int currentPosition = 1; // in index value


/* __________ Grid Functions __________*/

void initializeGrid()
{
  // Initialize Map
  // -1 for uninitialized, 0 for obstacles, and 1 for navigable space
  for(int i = 0; i < mapSize; i++)
  {
    for(int j = 0; j < mapSize; j++)
    {
      grid[i][j] = -1;
    }
  }
}


/* __________ Translate between Coordinates and Space Numbers __________*/

// Space Number to Coodinate
void spaceNumber_to_coord (int number) {
  // Returns two element array

  int j = ( (number-1) % mapSize);
  int i = ( (number - j - 1) / mapSize );

  spaceNumber_to_coord_Return[0] = i;
  spaceNumber_to_coord_Return[1] = j;
}
 


// Coordinate to Space Number
int coord_to_spaceNumber (int i, int j) {
  return ( (4 * i) + j + 1);
}



/* __________ Hard Code Grid __________*/

void hardCodeGrid()
{
  /*
   * Our Dijkstra's should produce an array:
   * If our robot is at (0,0) (Space 1), with goal (2,0) (Space 9) then the array would be:
   * Space Number:  [1, 2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16]
   * Array:         [2, 3,  7,  3,  X,  X,  11, 7,  9,  9, 10, 11,  9, 10,  X, 12]
   * 
   * 1 is navigable, 0 is obstacle
   * 
   * Grid:                Space Numbers
   *    0 1 2 3  =j       0   1   2   3  =j       
   *    
   * 0  1 1 1 1       0   1   2   3   4
   * 1  0 0 1 1       1   5   6   7   8
   * 2  1 1 1 1       2   9   10  11  12
   * 3  1 1 0 1       3   13  14  15  16
   * =                =
   * i                i
   * 
   */
   
  for(int j = 0; j < mapSize; j++)
  {
    grid[0][j] = 1;
    grid[2][j] = 1;
  }
  // second row
  grid[1][0] = 0;
  grid[1][1] = 0;
  grid[1][2] = 1;
  grid[1][3] = 1;

  // third row
  grid[3][0] = 1;
  grid[3][1] = 1;
  grid[3][2] = 0;
  grid[3][3] = 1;
}



/* __________ Cost Between Nodes __________*/   

// Cost between Our Position to another Position
int get_cost(int ourPos, int goalPos){

  int ourPosCoord[2];
  int goalPosCoord[2];

  spaceNumber_to_coord(ourPos);
  // coordinates [i,j]
  ourPosCoord[0] = spaceNumber_to_coord_Return[0];
  ourPosCoord[1] = spaceNumber_to_coord_Return[1];

  // coordinates [i,j]
  spaceNumber_to_coord(goalPos);
  goalPosCoord[0] = spaceNumber_to_coord_Return[0];
  goalPosCoord[1] = spaceNumber_to_coord_Return[1];
 
  int i_distance = abs(goalPosCoord[0] - ourPosCoord[0]);
  int j_distance = abs(goalPosCoord[1] - ourPosCoord[1]);

  /* adjacent nodes have a cost of one */
  if((i_distance + j_distance) > 1)
  {
    return INFINITY;
  }

  /* Same node */
  else if((goalPosCoord[0] == ourPosCoord[0]) && (goalPosCoord[1] == ourPosCoord[1]))
  {
    return 0;
  }
  else if((i_distance + j_distance) == 1)
  {
    /* Object in our way */
    if(grid[goalPosCoord[0]][goalPosCoord[1]] == 0)
    {
      return INFINITY;
    }
    /* Place is navigable */
    else if(grid[goalPosCoord[0]][goalPosCoord[1]] == 1)
    {
      return 1;
    }
  }
  /* I don't think we should ever get past here */
  else
  {
    return 0;
  }
  return -999;
}


/* __________ Dijkstras Functions __________*/


// Makes dijkstrasSolvedArray = [-1,-1,-1,...,-1]
void initializeDijkstrasSolvedArray()
{
  for(int iterator = 0;iterator < number_of_nodes;iterator++)
  {
    dijkstrasSolvedArray[iterator] = -1;
    // visitedFlag[iterator] = 0;
  }
}



void mattDijkstras(int startSpaceNumber, int finishSpaceNumber)
{
  /* Need to add:
   * 1. Go to object Edge Case.
   * 2. Go to non-existent Space Number Edge Case.
   * 3. 
   */

  int startIndex = startSpaceNumber - 1;  // Index is for array, space number is for our labeling
  int finishIndex = finishSpaceNumber -1; // Index is for array, space number is for our labeling
  initializeDijkstrasSolvedArray();       // Helper: set every element to -1

  /* Dijkstra's array solution will point to itself */
  dijkstrasSolvedArray[finishIndex] = finishSpaceNumber;
  
  /* Cylce through array while some -1's exist */
  int arraySolved = 0;

  while (arraySolved == 0)
  {
    arraySolved = 1;

    /* indexToEval goes from 0 to 15, corresponds to (spaceNumber - 1) */
    for (int indexToEval = 0; indexToEval < number_of_nodes; indexToEval++)
    {
      // int indexToEval = (mapSize*i + j);
      spaceNumber_to_coord(indexToEval + 1);
      /* Set value to infinity if we are an object, !!!move to its own loop to lessen computation time!!! */
      if (grid[spaceNumber_to_coord_Return[0]][spaceNumber_to_coord_Return[1]] == 0)
      {
        dijkstrasSolvedArray[indexToEval] = INFINITY;
      }

      if(dijkstrasSolvedArray[indexToEval] == -1)
      {
        arraySolved = 0;

        /* check LEFT node if it exists && is known */
        if ( (spaceNumber_to_coord_Return[1] > 0) && (dijkstrasSolvedArray[indexToEval-1] >= 0 ) && (dijkstrasSolvedArray[indexToEval-1] < INFINITY ) )
        {
          dijkstrasSolvedArray[indexToEval] = indexToEval;
        }
        /* check UPPER node if it exists && is known */
        else if ((spaceNumber_to_coord_Return[0] > 0) && (dijkstrasSolvedArray[indexToEval - mapSize] >= 0 ) && (dijkstrasSolvedArray[indexToEval - mapSize] < INFINITY ))
        {
          dijkstrasSolvedArray[indexToEval] = indexToEval - mapSize + 1;
        }

        /* check RIGHT node if it exists && is known */
        else if ( (spaceNumber_to_coord_Return[1] < (mapSize - 1) ) && (dijkstrasSolvedArray[indexToEval + 1] >= 0) && (dijkstrasSolvedArray[indexToEval + 1] < INFINITY ) )
        {
          dijkstrasSolvedArray[indexToEval] = (indexToEval + 2) ;
        }

        /* check LOWER node if it exists && is known */
        else if ( (spaceNumber_to_coord_Return[0] < (mapSize - 1)) && (dijkstrasSolvedArray[indexToEval + mapSize] >= 0 ) && (dijkstrasSolvedArray[indexToEval + mapSize] < INFINITY) )
        {
          dijkstrasSolvedArray[indexToEval] = (indexToEval + mapSize + 1);
        }
      }
    }
  }
}

void printSourcetoDestination(int currentSpaceNumber, int destSpaceNumber)
{
  mattDijkstras(currentSpaceNumber, destSpaceNumber);
  int iterator = currentSpaceNumber;
  int cost = 0;
  printf("\nPath from spaceNumber %d to spaceNumber %d is:\n", currentSpaceNumber, destSpaceNumber);
  while (iterator != destSpaceNumber)
  {
    cost++;
    printf("%d --> ", iterator);
    iterator = dijkstrasSolvedArray[iterator -1];
  }
  printf("%d\n",iterator);
  printf("Cost = %d\n", cost);
}




/* Test */
void printGrid()
{
  printf("Print our hardCodeGrid function, stored as 2-d array: \n");
  for(int i = 0; i < mapSize; i++)
  {
    for(int j = 0; j < mapSize; j++)
    {
       printf("%d ", grid[i][j]);
    }
    printf("\n");
  }
}

void printDijstrasSolvedArray()
{
  printf("Dijkstra's Solved Array, stores parent node:\n");
  for(int i = 0; i < mapSize; i++)
  {
    for(int j = 0; j < mapSize; j++)
    {
       printf("%d      ", dijkstrasSolvedArray[(mapSize*i) + j]);
    }
    printf("\n");
  }
}

void testFunction()
{
  // initializeGrid();
  // printf("Initialize Grid:");
  // printGrid();

  // hardCodeGrid();
  // printf("\nhardCodeGrid:");
  // printGrid();

  // initializeDijkstrasSolvedArray();
  // printf("Dijkstras Array Initialized:\n");
  // printDijstrasSolvedArray();

  // mattDijkstras(currentPosition, FINISHPOSITION);
  // printf("\nDijkstras Array Solved:\n");
  // printDijstrasSolvedArray();

  hardCodeGrid();
  printGrid();
  printSourcetoDestination(1,9);
  printDijstrasSolvedArray();
  printSourcetoDestination(4,9);
  printDijstrasSolvedArray();
  printSourcetoDestination(9,1);
  printDijstrasSolvedArray();
  printSourcetoDestination(16,16);
  printDijstrasSolvedArray();
  printSourcetoDestination(4,7);
  printDijstrasSolvedArray();
}











/* For Arduino Sparki: Setup */

// void setup() {
//   // Initialize output
  
//   sparki.servo(SERVO_CENTER);
//   initializeGrid();
 
//   // Current Position Initialization
//   currentPosition = 1;
 
//   // Time
//   loopTime = 100;
 
//   // Start it up
//   sparki.RGB(0,0,100);
//   delay(500);
//   sparki.RGB(0,0,0);
// }


/* For Arduino Sparki: Loop */

// void loop() {
//   // Set Start Time
//   startTime = millis();
  
//   // Wait until 100 ms
//   endTime = millis();
//   if (loopTime - (endTime - startTime) > 0)
//   {
//     delay(loopTime - (endTime - startTime));
//   }
//   else
//   {
//     sparki.RGB(100,0,0);
//   }
// }

int main(int argc, char const *argv[])
{
  /* code */
  testFunction();
  return 0;