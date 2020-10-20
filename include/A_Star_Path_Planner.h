#ifndef A_STAR_PATH_PLANNER_H
#define A_STAR_PATH_PLANNER_H

#include <vector>
#include <list>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "Map_Cell.h"



class A_Star_Path_Planner
{
    public:
        A_Star_Path_Planner( MapCell , MapCell,  std::vector<char> );

        float return_g_score( MapCell* ,unsigned int );
        float return_h_score( MapCell* );

        void add_neighbours( MapCell* );
        MapCell *get_best_neighbour( );

        void compute_path( MapCell* );
        void print_map();

        ~A_Star_Path_Planner();

        bool found_goal;

        bool DEBUG = true;



    private:


        const int map_width_ = 10;
        const int map_height_= 10;

        MapCell *start_;
        MapCell *goal_;

        // A star Search variables
        std::vector<MapCell> grid_; //main grid containing map cells
        std::vector<MapCell *> parent_; //layer array containing pointers to parent cell for each cell
        std::vector<MapCell *> path_; // layer array containing returned best path
        std::vector<unsigned int> neighbour_ids_; // layer array containing neihbours of current MapCell

        std::list<MapCell *> open_set_; //list of opened mapcells
        std::list<MapCell *> closed_set_; //list of closed mapcells

        std::vector<bool> opened_; //layer array contain opened flag for each map cell
        std::vector<bool> closed_; //layer array contain closed flag for each map cell

        std::vector<float> g_; //layer array containing g values for each map cell -->> dist covered
        std::vector<float> h_; //layer array containing h values for each map cell -->> dist to goal
        std::vector<float> f_; //layer array containing f values for each map cell -->> total cost

        std::vector<char> map_; //layer array representing OccupancyGrid map



};

#endif // A_STAR_PATH_PLANNER_H
