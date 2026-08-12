#pragma once

const int MAZE_WIDTH = 9;
const int MAZE_HEIGHT = MAZE_WIDTH;

enum direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

class Maze {
    void addWall(std::pair<int,int> pos, enum dir);
    void addWall(std::pair<float,float> pos, enum dir);
    void removeWall(std::pair<int,int> pos, enum dir);

    int vertical_walls[MAZE_WIDTH + 1];
    int horizontal_walls[MAZE_HEIGHT + 1];
};


