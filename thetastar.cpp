#include "thetastar.h"

Theta::~Theta(){}

bool Theta::lineofsight(int i1, int j1, int i2, int j2, const Map &map) {
    int di = std::abs(i2 - i1);

    int dj = std::abs(j2 - j1);

    int si, sj;

    int help = 0;

    if(i2 - i1 >= 0) {
        si = 1;
    } else {
        si = -1;
    }

    if(j2 - j1 >= 0) {
        sj = 1;
    } else {
        sj = -1;
    }

    if (di >= dj) {
        while(i1 != i2) {
            help += dj;

            if(help >= di) {
                if(map.CellIsObstacle(i1 + (si - 1)/2, j1 + (sj - 1)/2)) {
                    return false;
                }

                j1 += sj;

                help -= di;
            }

            if((help != 0 && map.CellIsObstacle(i1 + (si - 1)/2, j1 + (sj - 1)/2)) ||
                    (dj == 0 && map.CellIsObstacle(i1 + (si - 1)/2, j1) &&
                     map.CellIsObstacle(i1 + (si - 1)/2, j1 - 1))) {
                return false;
            }

            i1 += si;
        }
    } else {
        while(j1 != j2) {
            help += di;

            if(help >= dj) {
                if(map.CellIsObstacle(i1 + (si - 1)/2, j1 + (sj - 1)/2)) {
                    return false;
                }

                i1 += si;

                help -= di;
            }

            if((help != 0 && map.CellIsObstacle(i1 + (si - 1)/2, j1 + (sj - 1)/2)) ||
                    (dj == 0 && map.CellIsObstacle(i1, j1 + (sj - 1)/2) &&
                     map.CellIsObstacle(i1 - 1, j1 + (sj - 1)/2))) {
                return false;
            }

            j1 += sj;
        }
    }

    return true;
}

Node Theta::resetParent(Node curNode, Node parentNode, const Map &map) {
    if(parentNode.parent == nullptr) {
        return curNode;
    }
    if(lineofsight(parentNode.parent->i, parentNode.parent->j, curNode.i, curNode.j, map)) {
        curNode.parent = parentNode.parent;

        curNode.g = parentNode.parent->g + sqrt(pow(parentNode.parent->i - curNode.i, 2) + pow(parentNode.parent->j - curNode.j, 2));
    }

    return curNode;
}

void Theta::makePrimaryPath(Node pathNode)
{
    while(pathNode.parent) {
        lppath.push_front(pathNode);

        pathNode = *pathNode.parent;
    }

    lppath.push_front(pathNode);
}

void Theta::makeSecondaryPath()
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
