#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "astar.h"

class Dijkstra : public Astar
{
    public:
        Dijkstra();
        int computeHFromCellToCell(int start_i, int start_j, int finish_i, int finish_j, const EnvironmentOptions &options);
};
#endif
