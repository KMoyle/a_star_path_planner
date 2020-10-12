#ifndef A_STAR_PATH_PLANNER_H
#define A_STAR_PATH_PLANNER_H

#include <vector>
#include <cmath>

#include <Map_Cell.h>



class A_Star_Path_Planner
{
    public:
        A_Star_Path_Planner();
        virtual ~A_Star_Path_Planner();

    private:


    int map_width_;
    int map_height_;

    // A star Search variables
    std::vector<MapCell> grid_; //main grid containing map cells
    std::vector<MapCell *> parent_; //layer array containing pointers to parent cell for each cell

    std::vector<bool> opened_; //layer array contain opened flag for each map cell
    std::vector<bool> closed_; //layer array contain closed flag for each map cell

    std::vector<float> g_; //layer array containing g values for each map cell -->> dist covered
    std::vector<float> h_; //layer array containing h values for each map cell -->> dist to goal



};

#endif // A_STAR_PATH_PLANNER_H
