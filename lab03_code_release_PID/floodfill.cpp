#include <iostream>
#include <queue>

#define WIDTH 4
#define LENGTH 4



// cell data format
// int where    XXXG

int main() {
    int maze[WIDTH][LENGTH] = {};  // maze[x][y] bottom left is 0,0

    std::pair<int,int> startCell = std::make_pair(0,0);
    std::pair<int,int> goalCell = std::make_pair(3,3);

    std::cout << floodFill(maze, startCell, goalCell);

    return 0;
}

std::string floodFill(int maze[WIDTH][LENGTH], std::pair<int,int> startCell ,std::pair<int,int> goalCell) {
    int distances[WIDTH][LENGTH]{};
    std::queue<std::pair<int,int>> queue{};

    setGoal()
}

int setGoal() {
    
}

int setWall() {

}

int getNeighbours(int cell) {

}

