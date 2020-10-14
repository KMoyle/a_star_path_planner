#include <iostream>
#include "include/A_Star_Path_Planner.h"

using namespace std;

int main()
{

    MapCell start(0,0);
    MapCell goal(8,9);

    A_Star_Path_Planner AStar( start, goal );


    return 0;
}
