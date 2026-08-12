#include "maze.hpp"
#include "maze_data.h"

#include <Arduino.h>

namespace {

const float CELL_SIZE = 180.0f;

}  // namespace

void Maze::setWallBit(int& row, int bit, bool value) {
    if (value) {
        row |= (1 << bit);
    } else {
        row &= ~(1 << bit);
    }
}

Maze::Maze() {
    for (int k = 0; k < MAZE_SIZE; ++k) {
        vertical_walls[k] = 0;
        horizontal_walls[k] = 0;
    }
}

void Maze::addWall(std::pair<int, int> pos, direction dir) {
    const int i = pos.first;
    const int j = pos.second;

    if (dir == WEST || dir == EAST) {
        const int bit = (dir == WEST) ? i : i + 1;
        setWallBit(vertical_walls[j], bit, true);
    } else {
        const int bit = (dir == NORTH) ? j : j + 1;
        setWallBit(horizontal_walls[i], bit, true);
    }
}

void Maze::addWall(std::pair<float, float> pos, direction dir) {
    const int i = static_cast<int>(pos.first / CELL_SIZE);
    const int j = static_cast<int>(pos.second / CELL_SIZE);
    addWall({i, j}, dir);
}

void Maze::removeWall(std::pair<int, int> pos, direction dir) {
    const int i = pos.first;
    const int j = pos.second;

    if (dir == WEST || dir == EAST) {
        const int bit = (dir == WEST) ? i : i + 1;
        setWallBit(vertical_walls[j], bit, false);
    } else {
        const int bit = (dir == NORTH) ? j : j + 1;
        setWallBit(horizontal_walls[i], bit, false);
    }
}

void Maze::mazeLoad() {
    for (int k = 0; k < MAZE_SIZE; ++k) {
        vertical_walls[k] = MAZE_VERTICAL_WALLS[k];
        horizontal_walls[k] = MAZE_HORIZONTAL_WALLS[k];
    }
}

bool Maze::isWall(std::pair<int, int> pos, direction dir) const {
    const int i = pos.first;
    const int j = pos.second;

    if (dir == WEST || dir == EAST) {
        const int bit = (dir == WEST) ? i : i + 1;
        return (vertical_walls[j] >> bit) & 1;
    }

    const int bit = (dir == NORTH) ? j : j + 1;
    return (horizontal_walls[i] >> bit) & 1;
}

void Maze::printMaze() const {
    for (int j = 0; j < MAZE_SIZE; ++j) {
        Serial.print('+');
        for (int i = 0; i < MAZE_SIZE; ++i) {
            if (isWall({i, j}, NORTH)) {
                Serial.print("---");
            } else {
                Serial.print("   ");
            }
            Serial.print('+');
        }
        Serial.println();

        for (int i = 0; i < MAZE_SIZE; ++i) {
            Serial.print(isWall({i, j}, WEST) ? '|' : ' ');
            Serial.print(i);
            Serial.print(',');
            Serial.print(j);
        }
        Serial.println(isWall({MAZE_SIZE - 1, j}, EAST) ? '|' : ' ');
    }

    Serial.print('+');
    for (int i = 0; i < MAZE_SIZE; ++i) {
        if (isWall({i, MAZE_SIZE - 1}, SOUTH)) {
            Serial.print("---");
        } else {
            Serial.print("   ");
        }
        Serial.print('+');
    }
    Serial.println();
}
