#pragma once

#include "maze.hpp"
#include "waypoint.hpp"

enum class ExploreStep {
    MOVE,
    BACKTRACK,
    DONE
};

class MazePlanner {
public:
    explicit MazePlanner(Maze& maze);

    void resetExplore();
    ExploreStep getNextExploreMove(int ci, int cj, direction& outDir);
    void markVisited(int i, int j);
    int visitedCount() const;
    int visitPercent() const;
    bool isVisited(int i, int j) const;

    bool computeShortestPath(
        std::pair<int, int> start,
        std::pair<int, int> goal,
        Waypoint* outPath,
        int& outLen,
        int maxLen,
        float cellSizeMm
    ) const;

    static direction opposite(direction dir);
    static bool neighbour(
        int i,
        int j,
        direction dir,
        int& ni,
        int& nj
    );
    static Waypoint cellToWaypoint(int i, int j, float cellSizeMm);
    static float directionToHeading(direction dir);

private:
    Maze& maze_;
    bool visited_[MAZE_SIZE][MAZE_SIZE];
    direction moveStack_[MAX_SHORTEST_PATH];
    int stackSize_;
    int visitedCount_;

    bool canMove(int i, int j, direction dir) const;
};
