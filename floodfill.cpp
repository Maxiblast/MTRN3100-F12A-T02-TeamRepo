#include <iostream>
#include <queue>
#include <bitset>
#include <print>

#define WIDTH 4
#define HEIGHT 4

#define NORTH 0b0001
#define SOUTH 0b0010
#define EAST  0b0100
#define WEST  0b1000

#define GOAL   0b10000
#define START 0b100000

//Good thing for how to install g++ so you can compile & run this. 
//https://www.youtube.com/watch?v=1INYOToxdQY 

void setWall(int maze[WIDTH][HEIGHT], std::pair<int,int> cellA, std::pair<int,int> cellB);
void removeWall(int maze[WIDTH][HEIGHT], std::pair<int,int> cell, int dir);
int getDirTargetCell(std::pair<int,int> current, std::pair<int,int> target);
bool cellExists(std::pair<int,int> cell);
void printMaze(int maze[WIDTH][HEIGHT]);
bool hasWall(int cell, int dir);
bool hasGoal(int cell);
bool hasStart(int cell);
std::pair<int,int> getCellFromDir(std::pair<int,int> cell, int dir);
int getOppositeDir(int dir);
void setGoal(int maze[WIDTH][HEIGHT], std::pair<int,int> cell);
void setStart(int maze[WIDTH][HEIGHT], std::pair<int,int> cell);
void setAllWalls(int maze[WIDTH][HEIGHT]);

// cell data format
// int where XXSGWESN,      START,GOAL,WEST,EAST,SOUTH,NORTH

/*
 -- -- -- --        //13 char   3*n+1
|  |  |  |  |       //13 char
 -- -- -- --
|  |  |  |  |
 -- -- -- --
|  |  |  |  |
 -- -- -- --
|  |  |  |  |
 -- -- -- --
*/

int main() {
    int maze[WIDTH][HEIGHT] = {};  // maze[x][y] bottom left is 0,0
    
    setAllWalls(maze);
    printMaze(maze);

    std::cout << "------------------------------------" << std::endl;

    // removeWall(maze, std::make_pair(3,3), NORTH);
    removeWall(maze, std::make_pair(0,0), SOUTH);
    // removeWall(maze, std::make_pair(3,3), EAST);
    removeWall(maze, std::make_pair(0,0), WEST);

    setStart(maze, std::make_pair(0,0));
    setGoal(maze, std::make_pair(0,3));

    printMaze(maze);

    return 0;
}

std::string floodfill(int maze[WIDTH][HEIGHT]) {
    int distances[WIDTH][HEIGHT]{};

}

void setWall(int maze[WIDTH][HEIGHT], std::pair<int,int> cellA, std::pair<int,int> cellB) {
    if (cellExists(cellA) && cellExists(cellB)) {
        maze[cellA.first][cellA.second] = maze[cellA.first][cellA.second] | getDirTargetCell(cellA, cellB);
        maze[cellB.first][cellB.second] = maze[cellB.first][cellB.second] | getDirTargetCell(cellB, cellA);
    }
    else {
        std::cout << "Error Cell doesnt exist" << std::endl;
    }
}

void removeWall(int maze[WIDTH][HEIGHT], std::pair<int,int> cell, int dir) {
    if (cellExists(cell)) {
        maze[cell.first][cell.second] = maze[cell.first][cell.second] ^ dir; // turns OFF walls in direction
        std::pair<int,int> otherCell = getCellFromDir(cell, dir);
        if (cellExists(otherCell)) {
            maze[otherCell.first][otherCell.second] = maze[otherCell.first][otherCell.second] ^ getOppositeDir(dir);
        }   
    }
}

// returns the direction of target from current
// if cells arent adjacent it should give an error.
int getDirTargetCell(std::pair<int,int> current, std::pair<int,int> target) {
    if (!cellExists(current) || !cellExists(target)) {
        std::cout << "Error Cell doesnt exist" << std::endl; 
        return 0;
    }
    if (current.first == target.first && current.second + 1 == target.second) {
        return NORTH;
    }
    else if (current.first == target.first && current.second - 1 == target.second) {
        return SOUTH;
    }
    else if (current.first + 1 == target.first && current.second == target.second) {
        return EAST;
    }
    else if (current.first - 1 == target.first && current.second == target.second) {
        return WEST;
    }
    std::cout << "ERROR NO DIRECTION" << std::endl;
    return 0; // since 0 is 00000000 no walls will be made whe this is OR'd 
}

bool cellExists(std::pair<int,int> cell) {
    if (cell.first >= 0 && cell.first < WIDTH && cell.second >= 0 && cell.second < HEIGHT) {
        return true;
    }
    return false; 
}


void printMaze(int maze[WIDTH][HEIGHT]) {
    //start in the top left and print going down. 
    char output[3*WIDTH + 1][2*HEIGHT + 1];
    //init all pixels in output as ' '
    for (int j{0}; j < 2*HEIGHT + 1; j++) {
        for (int i{0}; i < 3*WIDTH + 1; i++) {
            output[i][j] = ' ';
        }
    }
    //pour maze into output
    for (int y{0}; y < HEIGHT; y++) {   
        for (int x{0}; x < WIDTH; x++) {
            //put walls in.
            if (hasWall(maze[x][y], NORTH)) {
                //north case
                output[3*x+1][2*y+2] = '-';
                output[3*x+2][2*y+2] = '-';
            }
            if (hasWall(maze[x][y], SOUTH)) {
                //south case
                output[3*x+1][2*y] = '-';
                output[3*x+2][2*y] = '-';
            }
            if (hasWall(maze[x][y], EAST)) {
                //east case                
                output[3*x+3][2*y+1] = '|';
            }
            if (hasWall(maze[x][y], WEST)) {
                //west case
                output[3*x][2*y+1] = '|';
            }
            //put goal and start in
            if (hasGoal(maze[x][y]) && hasStart(maze[x][y])) {
                std::cout << "ERROR has both goal and Start: " << x << y << std::endl;
                output[3*x+1][2*y+1] = 'G';
                output[3*x+2][2*y+1] = 'G';
            }
            else if (hasGoal(maze[x][y])) {
                output[3*x+1][2*y+1] = 'G';
                output[3*x+2][2*y+1] = 'G';
            }
            else if (hasStart(maze[x][y])) {
                output[3*x+1][2*y+1] = 'S';
                output[3*x+2][2*y+1] = 'S';
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////


    //print output
    for (int j{2*HEIGHT}; j >= 0 ; j--) {     // this line is weird with the + 1 
        for (int i{0}; i < 3*WIDTH + 1; i++) {
            std::cout << output[i][j];
        }
        // std::cout << ' ' << j << std::endl; //vertical char numbers for debugging
        std::cout << std::endl;
    }
}

/*

 Checks 
 if cell & dir == dir then wall exists.  
 E.g 0110 & 0010 == 0010.
 E.g 0101 & 0010 == 0000. 

*/  
bool hasWall(int cell, int dir) {
    return (cell & dir) == dir;     
}

bool hasGoal(int cell) {
    return (cell & GOAL) == GOAL;
}

bool hasStart(int cell) {
    return (cell & START) == START;
}

//warning returns self if error. 
std::pair<int,int> getCellFromDir(std::pair<int,int> cell, int dir) {
    std::pair<int,int> result = cell;
    switch (dir){
        case NORTH:
            result.second++;
        break;
        case SOUTH:
            result.second--;
        break;
        case EAST:
            result.first++;
        break; 
        case WEST:
            result.first--;
        break;
        default:
            std::cout << "getOppositeDir - Error not a direction";
        break;
    }
    return result;
}


// given a direction returns opposite direction
int getOppositeDir(int dir) {
    int result;
    switch (dir){
        case NORTH:
            result = SOUTH;
        break;
        case SOUTH:
            result = NORTH;
        break;
        case EAST:
            result = WEST;
        break; 
        case WEST:
            result = EAST;
        break;
        default:
            std::cout << "getOppositeDir - Error not a direction";
            result = 0;
        break;
    }
    return result;
}

void setGoal(int maze[WIDTH][HEIGHT], std::pair<int,int> cell) {
    if (cellExists(cell)) {
        maze[cellA.first][cellA.second] = maze[cellA.first][cellA.second] | GOAL;
    }
    else {
        std::cout << "Error Cell doesnt exist" << std::endl;
    }
}

void setStart(int maze[WIDTH][HEIGHT], std::pair<int,int> cell) {
    if (cellExists(cell)) {
        maze[cellA.first][cellA.second] = maze[cellA.first][cellA.second] | START;
    }
    else {
        std::cout << "Error Cell doesnt exist" << std::endl;
    }
}

//fills the maze with walls
void setAllWalls(int maze[WIDTH][HEIGHT]) {
    for (int y{0}; y < HEIGHT; y++) {
        for (int x{0}; x < WIDTH; x++) {
            maze[x][y] = maze[x][y] | 0b1111; 
        }
    }
}