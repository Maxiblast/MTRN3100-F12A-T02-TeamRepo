#include <iostream>
#include <queue>
#include <bitset>

#define WIDTH 4
#define LENGTH 4

#define NORTH 0b0001
#define SOUTH 0b0010
#define EAST  0b0100
#define WEST  0b1000

//Good thing for how to install g++ so you can run this. 
//https://www.youtube.com/watch?v=1INYOToxdQY 

// std::string floodFill(int maze[WIDTH][LENGTH], std::pair<int,int> startCell ,std::pair<int,int> goalCell);

// int setGoal(); 

void setWall(int maze[WIDTH][LENGTH], std::pair<int,int> cellA, std::pair<int,int> cellB);

int getDirTargetCell(std::pair<int,int> current, std::pair<int,int> target);

// int getNeighbours(int cell);

bool cellExists(std::pair<int,int> cell);


// cell data format
// int where XXXGWESN

int main() {
    int maze[WIDTH][LENGTH] = {};  // maze[x][y] bottom left is 0,0

    setWall(maze, std::make_pair(0,1),std::make_pair(0,2));
    std::cout << "cell A walls: " << std::bitset<8>(maze[0][1]) << std::endl;
    std::cout << "cell B walls: " << std::bitset<8>(maze[0][2]) << std::endl;
    // std::pair<int,int> startCell = std::make_pair(0,0);
    // std::pair<int,int> goalCell = std::make_pair(3,3);

    // std::cout << floodFill(maze, startCell, goalCell);

    return 0;
}

// std::string floodFill(int maze[WIDTH][LENGTH], std::pair<int,int> startCell ,std::pair<int,int> goalCell) {
//     int distances[WIDTH][LENGTH] ={};
//     std::queue<std::pair<int,int>> queue{};
// }

// int setGoal() {
    
// }


//sets a wall between cellA and cellB 
//if walls are not adjacent it shouldnt do anything. despite not checking if they are adjacent or not. 
void setWall(int maze[WIDTH][LENGTH], std::pair<int,int> cellA, std::pair<int,int> cellB) {
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

// int getNeighbours(int cell) {

// }

bool cellExists(std::pair<int,int> cell) {
    if (cell.first >= 0 && cell.first < WIDTH && cell.second >= 0 && cell.second < LENGTH) {
        return true;
    }
    return false; 
}
