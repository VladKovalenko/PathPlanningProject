#include "isearch.h"

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}

int ISearch::findopen(Node node){
    for(int i = 0; i < open.size(); ++i){
        if(open[i] == node) {
            return i;
        }
    }

    return -1;
}


SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

    Node pathNODE;

    pathNODE.parent = nullptr;

    pathNODE.i = map.getstarti();

    pathNODE.j = map.getstartj();

    pathNODE.g = 0;

    pathNODE.H = computeHFromCellToCell(map.getstarti(), map.getstartj(), map.getgoali(), map.getgoalj(), options);

    pathNODE.F = pathNODE.H * hweight;

    bool done = 0;

    open.push_back(pathNODE);

    while(!open.empty()) {
        pathNODE = findMinNode();

        close.insert(std::make_pair(pathNODE.i * map.getMapWidth() + pathNODE.j, pathNODE));

        if (findopen(pathNODE) != -1) {
            open.erase(open.begin() + findopen(pathNODE));
        }

        Node curGoal;

        curGoal.i = map.getgoali();

        curGoal.j = map.getgoalj();

        if (curGoal == pathNODE) {
            done = 1;

            break;
        } else {
            std::list<Node> curSuccessors = findSuccessors(pathNODE, map, options);

            std::list<Node>::iterator curIter = curSuccessors.begin();

            while(curIter != curSuccessors.end()) {
                (*curIter).parent = &(close.find(pathNODE.i * map.getMapWidth() + pathNODE.j)->second);

                *curIter = resetParent(*curIter, *((*curIter).parent), map);

                (*curIter).H = computeHFromCellToCell((*curIter).i, (*curIter).j, map.getgoali(), map.getgoalj(), options);

                (*curIter).F =  hweight * (*curIter).H + (*curIter).g;

                if(findopen(*curIter) != -1) {
                    if(open[findopen(*curIter)].F > (*curIter).F) {
                        open[findopen(*curIter)] = (*curIter);
                    } else if(open[findopen(*curIter)].F == (*curIter).F){
                        if(breakingties == CN_SP_BT_GMAX && open[findopen(*curIter)].g > (*curIter).g){
                            open[findopen(*curIter)] = (*curIter);
                        } else if(breakingties == CN_SP_BT_GMIN && open[findopen(*curIter)].g < (*curIter).g){
                            open[findopen(*curIter)] = (*curIter);
                        }
                    }
                } else {
                    open.push_back(*curIter);
                }

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

    minNode.F = std::numeric_limits<double>::infinity();

    for (int i = 0; i < open.size(); i++) {
        if(open[i].F < minNode.F){
            minNode = open[i];
        } else if(open[i].F == minNode.F){
            if(breakingties == CN_SP_BT_GMAX && open[i].g > minNode.g){
                minNode = open[i];
            } else if(breakingties == CN_SP_BT_GMIN && open[i].g < minNode.g){
                minNode = open[i];
            }
        }
    }

    return minNode;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    std::list<Node> successors;

    Node successor;

    for(int i = -1; i < 2; ++i) {
        for(int j = -1; j < 2; ++j) {
            if(!(i == 0 && j == 0)) {
                if(map.CellOnGrid(curNode.i + i, curNode.j + j) && map.CellIsTraversable(curNode.i + i, curNode.j + j)) {
                    if((i == 0 || j == 0) || (i!= 0 && j != 0 && (options.allowdiagonal || (options.allowsqueeze &&
                                                                     map.CellIsObstacle(curNode.i, curNode.j + j) &&
                                                     map.CellIsObstacle(curNode.i + i, curNode.j)) ||
                                (options.cutcorners && (map.CellIsObstacle(curNode.i, curNode.j + j)
                                                        || map.CellIsObstacle(curNode.i + i, curNode.j)))))) {
                        if(close.find((curNode.i + i) * map.getMapWidth() + (curNode.j + j)) == close.end()) {
                            successor.i = curNode.i + i;

                            successor.j = curNode.j + j;

                            if(i == 0 || j == 0) {
                                successor.g = curNode.g + 1;
                            } else {
                                successor.g = curNode.g + sqrt(2);
                            }

                                successors.push_back(successor);
                        }
                    }
                }
            }
        }
    }

    return successors;
}

void ISearch::makePrimaryPath(Node pathNode)
{
    while(pathNode.parent) {
        lppath.push_front(pathNode);

        pathNode = *pathNode.parent;
    }

    lppath.push_front(pathNode);
}

void ISearch::makeSecondaryPath()
{
    std::list<Node>::const_iterator curIter = lppath.begin();

    hppath.push_front(*curIter);

    int presx, presy, middlex, middley, nextx, nexty;

    std::list<Node>::const_iterator lastIter = lppath.end();

    while(curIter != lastIter) {
        presx = curIter->i;

        presy = curIter->j;

        ++curIter;

        middlex = curIter->i;

        middley = curIter->j;

        ++curIter;

        nextx = curIter->i;

        nexty = curIter->j;

        --curIter;

        if(nextx - middlex != middlex - presx || nexty - middley != middley - presy) {
            hppath.push_front(*curIter);
        }
    }
}
