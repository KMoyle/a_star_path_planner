#include <iostream>
#include "include/A_Star_Path_Planner.h"

using namespace std;

int main()
{

    MapCell start(0,0);
    MapCell goal(9,9);

    //creating a row major vector to emulate the ROS nav_msgs/OccupancyGrid
    std::vector<int> map_( 10 * 10 );
    //assigning free space and obs -->> making an obs arc around the goal
    for( unsigned int y = 0; y <  10; y++ ){
        for( unsigned int x = 0; x <  10; x++ ){
            if( x == 8 && y > 4 &&  y < 9 ){
               map_[ x + y * 10 ] = 0;
            } else if ( y == 8 && x > 4 &&  x < 9 ){
                map_[ x + y * 10 ] = 0;
            } else {
                map_[ x + y * 10 ] = 1;
            }
        }
    }



    A_Star_Path_Planner AStar( start, goal, map_ );


    return 0;
}
