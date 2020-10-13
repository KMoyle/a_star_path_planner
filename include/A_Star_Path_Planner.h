#ifndef A_STAR_PATH_PLANNER_H
#define A_STAR_PATH_PLANNER_H

#include <vector>
#include <list>
#include <cmath>
#include <iostream>

#include "Map_Cell.h"



class A_Star_Path_Planner
{
    public:
        A_Star_Path_Planner( MapCell , MapCell );

        float return_g_score( MapCell* , MapCell* );
        float return_h_score( MapCell* );

        void add_neighbours( MapCell* );
        float get_best_neighbour( float );

        bool found_goal;

        virtual ~A_Star_Path_Planner();

    private:


        const int map_width_ = 10;
        const int map_height_= 10;

        MapCell *start_;
        MapCell *goal_;

        // A star Search variables
        std::vector<MapCell> grid_; //main grid containing map cells
        std::vector<MapCell *> parent_; //layer array containing pointers to parent cell for each cell
        std::vector<MapCell *> path_; // layer array containing returned best path
        std::vector<MapCell *> neighbours_; // layer array containing neihbours of current MapCell

        std::list<MapCell *> open_set_; //list of opened mapcells
        std::list<MapCell *> closed_set_; //list of closed mapcells

        std::vector<bool> opened_; //layer array contain opened flag for each map cell
        std::vector<bool> closed_; //layer array contain closed flag for each map cell

        std::vector<float> g_; //layer array containing g values for each map cell -->> dist covered
        std::vector<float> h_; //layer array containing h values for each map cell -->> dist to goal
        std::vector<float> f_; //layer array containing f values for each map cell -->> total cost



};

#endif // A_STAR_PATH_PLANNER_H
