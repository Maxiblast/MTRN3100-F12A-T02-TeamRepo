#pragma once

// #include <utility>

const int MAZE_SIZE = 9;

enum direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

class Maze {
public:
    Maze();
    
    void addWall(std::pair<int, int> pos, direction dir);
    void addWall(std::pair<float, float> pos, direction dir);
    void removeWall(std::pair<int, int> pos, direction dir);
    bool isWall(std::pair<int, int> pos, direction dir) const;
    void printMaze() const;
    void mazeLoad();

private:
    int vertical_walls[MAZE_SIZE];
    int horizontal_walls[MAZE_SIZE];
    void setWallBit(int& row, int bit, bool value);
};
