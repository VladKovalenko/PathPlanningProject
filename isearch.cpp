#include "isearch.h"
#include <chrono>
#include <math.h>
#include <limits>


ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}


SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

    Node pathNODE;

    pathNODE.parent = nullptr;

    pathNODE.i = map.start_i;

    pathNODE.j = map.start_j;

    pathNODE.g = 0;

    pathNODE.H = computeHFromCellToCell(map.start_i, map.start_j, map.goal_i, map.goal_j, &options);

    pathNODE.F = pathNODE.h * hweight;

    bool done = 0;

    while(!open.empty) {
        pathNODE = findMinNode();

        close.insert({curNode.i * map.width + curNode.j, curNode});

        open.erase(open.find(curNode.i * map.width + curNode.j));

        Node curGoal;

        curGoal.i = map.goal_i;

        curGoal.j = map.goal_j;

        if (curGoal = pathNODE) {
            done = 1;

            break;
        } else {
            std::list<Node> curSuccessors = findSuccessors(pathNODE, map, options);

            std::list<Node>::iterator curIter = curSuccessors.begin();

            while(curIter != curSuccessors.end()) {
                (*curIter).parent = &(close.find(curNode.i * map.width + curNode.j)->second);

                (*curIter).H = computeHFromCellToCell((*curIter).i, (*curIter).j, map.goal_i, map.goal_j, options);

                (*curIter).F =  hweight * (*curIter).H + (*curIter).g;

                open.insert({(*curIter).i * map.width + (*curIter).j, *curIter})

                ++curIter;
            }
        }
    }

    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1000000000;
    
    sresult.nodescreated =  open.size() + close.size();
    
    sresult.numberofsteps = close.size();
    if(done) {
        sresult.pathfound = 1;
        
        makePrimaryPath(pathNODE);

        sresult.pathlength = pathNODE.g;

        makeSecondaryPath();
    }
    
    sresult.hppath = &hppath; 
    
    sresult.lppath = &lppath;
    
    return sresult;
}

Node ISearch::findMinNode() {
    Node minNode;

    min.F = std::numeric_limits<double>::infinity();
    
    for (int i = 0; i < open.size(); i++) {
        if ((!open[i].empty() && open[i].begin()->F == min.F && ((breakingties == CN_SP_BT_GMAX && open[i].begin()->g >= min.g) ||
            (breakingties == CN_SP_BT_GMIN && open[i].begin()->g <= min.g))) || (!open[i].empty() && open[i].begin()->F < min.F)) {
            min = *open[i].begin();
    }
    
    return minNode;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    std::list<Node> successors;

    Node successor;
    
    for(int i = -1; i < 2; ++i) {
        for(int j = -1; j < 2; ++i) {
            if((i != 0 || j != 0) && !map.CellIsObstacle(curNode.i + i, curNode.j + j) &&
             map.CellOnGrid(curNode.i + i, curNode.j + j)) {
                if(i != 0 && j != 0 && !options.allowdiagonal) {
                    continue;
                } else if(i != 0 && j != 0 && !options.cutcorners &&
                 map.CellIsObstacle(curNode.i, curNode.j + j) || 
                 map.CellIsObstacle(curNode.i + i, curNode.j)) {
                    continue;
                 } else if(i != 0 && j != 0 && !options.allowsqueeze &&
                 map.CellIsObstacle(curNode.i, curNode.j + j) && 
                 map.CellIsObstacle(curNode.i + i, curNode.j)) {
                    continue;
                 } else if (close.find((curNode.i + i) * map.width + curNode.j + j) == close.end()){
                    successor.i = curNode.i + i;

                    successor.j = curNode.j + j;

                    if(i != 0 && j != 0) {
                        successor.g = curNode.g + sqrt(2);
                    } else {
                        successor.g = curNode.g + 1;
                    }

                    successors.push_back(successor);
                 }
            }
        }
    }

    return successors;
}

void ISearch::makePrimaryPath(Node curNode) {
    Node pathNode;

    curNode = pathNode;

    while(pathNode.parent) { 
        lppath.push_front(pathNode);

        moveNode = *pathNode.parent;
    }

    lppath.push_front(pathNode);
}

void ISearch::makeSecondaryPath() {
    std::list<Node>::const_iterator curIter = lppath.begin();

    hppath.push_front(*curIter);

    int presx, presy, middlex, middley, nextx, nexty;

    std::list<Node>::const_iterator lastIter = lppath.end();

    while(curIter != lastIter) {
        presx = *curIter.i;

        presy = *curIter.j;

        ++curIter;

        middlex = *curIter.i;

        middley = *curIter.j;

        ++curIter;

        nextx = *curIter.i;

        nexty = *curIter.j;

        --curIter;

        if((presx - middlex == 0 && middlex - nextx != 0 && presy - middley != 0 && middley - nexty == 0) ||
            (presx - middlex != 0 && middlex - nextx == 0 && presy - middley == 0 && middley - nexty != 0)) {
            hppath.push_front(*curIter);
        }
    }
}
