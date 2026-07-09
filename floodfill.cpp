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
int getDirTargetCell(std::pair<int,int> current, std::pair<int,int> target);
bool cellExists(std::pair<int,int> cell);
void printMaze(int maze[WIDTH][HEIGHT]);
bool hasWall(int cell, int dir);
bool hasGoal(int cell);
bool hasStart(int cell);

// cell data format
// int where XXXGWESN

int main() {
    int maze[WIDTH][HEIGHT] = {};  // maze[x][y] bottom left is 0,0

    for (int y{0}; y < HEIGHT - 1; y++) {
        for (int x{0}; x < WIDTH - 1; x++) {
            maze[x][y] = maze[x][y] | 0b1111; 
        }
    }

    setWall(maze, std::make_pair(0,1),std::make_pair(0,2));
    setWall(maze, std::make_pair(3,0),std::make_pair(3,1));
    setWall(maze, std::make_pair(0,0),std::make_pair(0,1));
    printMaze(maze);



    return 0;
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
    for (int y{0}; y < HEIGHT - 1; y++) {
        for (int x{0}; x < WIDTH - 1; x++) {
            //put empty corner spaces in.
            // output[3*x][2*y] = ' ';
            // output[3*x][2*y+2] = ' ';
            // output[3*x+3][2*y] = ' ';
            // output[3*x+3][2*y+2] = ' ';
            // //put center empty's in.

            // output[3*x+1][2*y+1] = ' ';
            // output[3*x+2][2*y+1] = ' ';

            // if (hasGoal(maze[x][y]) && hasStart(maze[x][y])) {
            //     std::cout << "ERROR has both goal and Start";
            //     output[3*x+1][2*y+1] = 'G';
            //     output[3*x+2][2*y+1] = 'G';
            // }
            // else if (hasGoal(maze[x][y])) {
            //     output[3*x+1][2*y+1] = 'G';
            //     output[3*x+2][2*y+1] = 'G';
            // }
            // else if (hasStart(maze[x][y])) {
            //     output[3*x+1][2*y+1] = 'S';
            //     output[3*x+2][2*y+1] = 'S';
            // } else {
            //     output[3*x+1][2*y+1] = ' ';
            //     output[3*x+2][2*y+1] = ' ';
            // }

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
        }
    }

    //print output
    for (int j{2*HEIGHT}; j > 0 ; j--) {
        for (int i{0}; i < 3*WIDTH + 1; i++) {
            std::cout << output[i][j];
        }
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
    return cell & dir == dir;     
}

bool hasGoal(int cell) {
    return cell & GOAL == GOAL;
}

bool hasStart(int cell) {
    return cell & START == START;
}

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



  --  --  --
||  ||  ||  ||
  --  --  -- 
||  ||  ||  ||
  --  --  --
||  ||  ||  ||
  --  --  --

*/


