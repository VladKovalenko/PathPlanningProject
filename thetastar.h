#ifndef THETASTAR_H
#define THETASTAR_H
#include "astar.h"


class Theta: public Astar {
    public:
        Theta(double hweight, bool breakingties):Astar(hweight, breakingties){}

        ~Theta();
    protected:
        bool lineofsight(int i1, int j1, int i2, int j2, const Map &map);

        Node resetParent(Node curNode, Node parentNode, const Map &map);

        void makePrimaryPath(Node curNode);

        void makeSecondaryPath();
};
#endif
