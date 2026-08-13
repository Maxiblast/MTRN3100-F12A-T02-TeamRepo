// #include "maze_planner.hpp"

// #include <Arduino.h>

// namespace {

// const direction kSearchOrder[] = {NORTH, EAST, SOUTH, WEST};

// }  // namespace

// MazePlanner::MazePlanner(Maze& maze)
//     : maze_(maze), stackSize_(0), visitedCount_(0) {
//     resetExplore();
// }

// void MazePlanner::resetExplore() {
//     for (int j = 0; j < MAZE_SIZE; ++j) {
//         for (int i = 0; i < MAZE_SIZE; ++i) {
//             visited_[j][i] = false;
//         }
//     }
//     stackSize_ = 0;
//     visitedCount_ = 0;
// }

// void MazePlanner::markVisited(int i, int j) {
//     if (i < 0 || j < 0 || i >= MAZE_SIZE || j >= MAZE_SIZE) {
//         return;
//     }
//     if (!visited_[j][i]) {
//         visited_[j][i] = true;
//         ++visitedCount_;
//     }
// }

// int MazePlanner::visitedCount() const {
//     return visitedCount_;
// }

// int MazePlanner::visitPercent() const {
//     return (visitedCount_ * 100) / (MAZE_SIZE * MAZE_SIZE);
// }

// bool MazePlanner::isVisited(int i, int j) const {
//     if (i < 0 || j < 0 || i >= MAZE_SIZE || j >= MAZE_SIZE) {
//         return false;
//     }
//     return visited_[j][i];
// }

// direction MazePlanner::opposite(direction dir) {
//     switch (dir) {
//         case NORTH:
//             return SOUTH;
//         case EAST:
//             return WEST;
//         case SOUTH:
//             return NORTH;
//         case WEST:
//             return EAST;
//     }
//     return NORTH;
// }

// bool MazePlanner::neighbour(
//     int i,
//     int j,
//     direction dir,
//     int& ni,
//     int& nj
// ) {
//     ni = i;
//     nj = j;
//     switch (dir) {
//         case NORTH:
//             --nj;
//             break;
//         case EAST:
//             ++ni;
//             break;
//         case SOUTH:
//             ++nj;
//             break;
//         case WEST:
//             --ni;
//             break;
//     }
//     return ni >= 0 && nj >= 0 && ni < MAZE_SIZE && nj < MAZE_SIZE;
// }

// Waypoint MazePlanner::cellToWaypoint(int i, int j, float cellSizeMm) {
//     Waypoint waypoint;
//     waypoint.x = (static_cast<float>(i) + 0.5f) * cellSizeMm;
//     waypoint.y = (static_cast<float>(j) + 0.5f) * cellSizeMm;
//     return waypoint;
// }

// float MazePlanner::directionToHeading(direction dir) {
//     switch (dir) {
//         case NORTH:
//             return -90.0f;
//         case EAST:
//             return 0.0f;
//         case SOUTH:
//             return 90.0f;
//         case WEST:
//             return 180.0f;
//     }
//     return 0.0f;
// }

// bool MazePlanner::canMove(int i, int j, direction dir) const {
//     int ni = 0;
//     int nj = 0;
//     if (!neighbour(i, j, dir, ni, nj)) {
//         return false;
//     }
//     if (visited_[nj][ni]) {
//         return false;
//     }
//     return !maze_.isWall({i, j}, dir);
// }

// ExploreStep MazePlanner::getNextExploreMove(int ci, int cj, direction& outDir) {
//     markVisited(ci, cj);

//     for (direction dir : kSearchOrder) {
//         if (!canMove(ci, cj, dir)) {
//             continue;
//         }

//         if (stackSize_ < MAX_SHORTEST_PATH) {
//             moveStack_[stackSize_++] = dir;
//         }
//         outDir = dir;
//         return ExploreStep::MOVE;
//     }

//     if (stackSize_ > 0) {
//         outDir = opposite(moveStack_[--stackSize_]);
//         return ExploreStep::BACKTRACK;
//     }

//     return ExploreStep::DONE;
// }

// bool MazePlanner::computeShortestPath(
//     std::pair<int, int> start,
//     std::pair<int, int> goal,
//     Waypoint* outPath,
//     int& outLen,
//     int maxLen,
//     float cellSizeMm
// ) const {
//     const int startI = start.first;
//     const int startJ = start.second;
//     const int goalI = goal.first;
//     const int goalJ = goal.second;

//     if (maxLen <= 0 || outPath == nullptr) {
//         outLen = 0;
//         return false;
//     }

//     bool discovered[MAZE_SIZE][MAZE_SIZE] = {};
//     int parentI[MAZE_SIZE][MAZE_SIZE];
//     int parentJ[MAZE_SIZE][MAZE_SIZE];

//     int queueI[MAX_SHORTEST_PATH];
//     int queueJ[MAX_SHORTEST_PATH];
//     int head = 0;
//     int tail = 0;

//     queueI[tail] = startI;
//     queueJ[tail] = startJ;
//     ++tail;
//     discovered[startJ][startI] = true;
//     parentI[startJ][startI] = startI;
//     parentJ[startJ][startI] = startJ;

//     while (head < tail) {
//         const int ci = queueI[head];
//         const int cj = queueJ[head];
//         ++head;

//         if (ci == goalI && cj == goalJ) {
//             break;
//         }

//         for (direction dir : kSearchOrder) {
//             int ni = 0;
//             int nj = 0;
//             if (!neighbour(ci, cj, dir, ni, nj)) {
//                 continue;
//             }
//             if (maze_.isWall({ci, cj}, dir) || discovered[nj][ni]) {
//                 continue;
//             }

//             discovered[nj][ni] = true;
//             parentI[nj][ni] = ci;
//             parentJ[nj][ni] = cj;

//             if (tail < MAX_SHORTEST_PATH) {
//                 queueI[tail] = ni;
//                 queueJ[tail] = nj;
//                 ++tail;
//             }
//         }
//     }

//     if (!discovered[goalJ][goalI]) {
//         outLen = 0;
//         return false;
//     }

//     int cellsI[MAX_SHORTEST_PATH];
//     int cellsJ[MAX_SHORTEST_PATH];
//     int pathLen = 0;

//     int ci = goalI;
//     int cj = goalJ;
//     while (pathLen < MAX_SHORTEST_PATH) {
//         cellsI[pathLen] = ci;
//         cellsJ[pathLen] = cj;
//         ++pathLen;

//         if (ci == startI && cj == startJ) {
//             break;
//         }

//         const int nextI = parentI[cj][ci];
//         const int nextJ = parentJ[cj][ci];
//         ci = nextI;
//         cj = nextJ;
//     }

//     if (pathLen > maxLen) {
//         outLen = 0;
//         return false;
//     }

//     outLen = pathLen;
//     for (int k = 0; k < pathLen; ++k) {
//         const int reversed = pathLen - 1 - k;
//         outPath[k] = cellToWaypoint(
//             cellsI[reversed],
//             cellsJ[reversed],
//             cellSizeMm
//         );
//     }

//     return true;
// }
