#pragma once

#include "Cell.h"
#include "Options.h"

#if (SIMULATOR)
#include "../../IAlgorithm.h"
#endif

namespace mack {

static const int MAZE_WIDTH = 9;  // XXX
static const int MAZE_HEIGHT = 9; // XXX
enum {NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3};

#if (SIMULATOR)
class MackAlgo : IAlgorithm {
#else
class MackAlgo {
#endif

public:
#if (SIMULATOR)
    void solve(sim::MouseInterface* mouse);
#else
    void solve();
#endif

    Cell **getMazeWalls();

private:
#if (SIMULATOR)
    sim::MouseInterface* m_mouse;
#endif
    Cell m_maze[MAZE_WIDTH][MAZE_HEIGHT];
    int m_x;
    int m_y;
    int m_d;
    bool m_onWayToCenter;
    int m_moveBufferIndex;

    void move();
    float getTurnCost();
    float getStraightAwayCost(int length);
    bool inspectNeighbor(Cell* current, Cell* neighbor, int direction);

    void initializeDestinationDistance();
    Cell* getClosestDestinationCell();
    Cell* cellMin(Cell* one, Cell* two);

    void readWalls();
    bool inGoal(int x, int y);

    void turnLeftUpdateState();
    void turnRightUpdateState();
    void turnAroundUpdateState();
    void moveForwardUpdateState();

#if (SIMULATOR)
    void turnLeft();
    void turnRight();
    void turnAround();
#endif

    void moveForward();
    void leftAndForward();
    void rightAndForward();
    void aroundAndForward();

    Cell* getFrontCell();
    Cell* getLeftCell();
    Cell* getRightCell();
    Cell* getRearCell();
    bool spaceFront();
    bool spaceLeft();
    bool spaceRight();

    bool isOneCellAway(Cell* target);
    void moveOneCell(Cell* target);

#if (SIMULATOR)
    void setColor(int x, int y, char color);
    void resetColors();
    void colorCenter(char color);
#endif

};

} // namespace mack
