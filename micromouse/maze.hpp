#pragma once

#include "waypoint.hpp"

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
    
    void addWall(Waypoint pos, direction dir);
    void removeWall(Waypoint pos, direction dir);
    bool isWall(Waypoint pos, direction dir) const;
    void printMaze() const;
    void mazeLoad();

private:
    int vertical_walls[MAZE_SIZE];
    int horizontal_walls[MAZE_SIZE];
    void setWallBit(int& row, int bit, bool value);
};
