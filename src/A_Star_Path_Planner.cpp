#include "../include/A_Star_Path_Planner.h"

A_Star_Path_Planner::A_Star_Path_Planner( MapCell start, MapCell goal )
{
    start_ = &start;
    goal_ = &goal;

    //init sizes of all a* vecs
    grid_.resize( map_height_ * map_width_ );
    parent_.resize( map_height_ * map_width_ );
    neighbours_.resize( map_height_ * map_width_ );
    path_.reserve( map_height_ * map_width_ );
    open_set_.clear();
    closed_set_.clear();
    opened_.resize( map_height_ * map_width_ );
    closed_.resize( map_height_ * map_width_ );
    g_.resize( map_height_ * map_width_ );
    h_.resize( map_height_ * map_width_ );
    f_.resize( map_height_ * map_width_ );

    //initialise grid
    for( unsigned int y =0; y < static_cast<unsigned int>( map_height_ ); y++){
        for( unsigned int x =0; x < static_cast<unsigned int>( map_width_ ); x++){

            grid_[ x + y * map_width_ ].x( x );
            grid_[ x + y * map_width_ ].y( y );

        }
    }

    //init lists with start mapcell
    open_set_.push_back( start_ );
    opened_[ start_->get_x() + start_->get_y() * map_width_ ] = true;
    g_[ start_->get_x() + start_->get_y() * map_width_ ] = 0;
    h_[ start_->get_x() + start_->get_y() * map_width_ ] = return_h_score( start_ );
    f_[ start_->get_x() + start_->get_y() * map_width_ ] = g_[ start_->get_x() + start_->get_y() * map_width_ ] +  h_[ start_->get_x() + start_->get_y() * map_width_ ];


    //start search
    while ( !open_set_.empty() ){

        MapCell *current = open_set_.front();

        if ( current->get_x() == goal_->get_x() && current->get_x() == goal_->get_x() ){
            found_goal = true;
            std::cout << " FOUND PATH " << std::endl;
            break;
        }
        //remove current point from opened set and place in closed set and set closed to true
        closed_set_.push_back( open_set_.front() );
        closed_[ current->get_x() + current->get_y() * map_width_ ] = true;
        open_set_.pop_front();


        //get valid neighbours of current i.e. within bounds
        neighbours_.clear();
        add_neighbours( current );







    }





    //ctor
}

float A_Star_Path_Planner::return_g_score( MapCell *cmp, MapCell *nmc){

    return 0;
}

float A_Star_Path_Planner::return_h_score( MapCell *mp ){

    return abs( mp->get_x() - goal_->get_x() ) + abs( mp->get_y() - goal_->get_y() );

}

void A_Star_Path_Planner::add_neighbours( MapCell *cmp ){
    // TODO add robustness for inf or obs for real maps
    MapCell *n1;

    //for x dir
    if ( cmp->get_x() - 1 >= 0 && !closed_[ ( cmp->get_x()-1 ) + cmp->get_y() * map_width_] ){
        n1->x( cmp->get_x() - 1 );
        n1->y( cmp->get_y() );
        neighbours_.push_back( n1 );
    }
    if ( cmp->get_x() + 1 < map_width_ && !closed_[ ( cmp->get_x()+1 ) + cmp->get_y() * map_width_]){
        n1->x( cmp->get_x() + 1 );
        n1->y( cmp->get_y() );
        neighbours_.push_back( n1 );
    }
    //for y dir
    if ( cmp->get_y() - 1 >= 0 && !closed_[ cmp->get_x() + (cmp->get_y() - 1) * map_width_] ){
        n1->x( cmp->get_x() - 1 );
        n1->y( cmp->get_y() );
        neighbours_.push_back( n1 );
    }
    if ( cmp->get_y() + 1 < map_height_ && !closed_[ cmp->get_x() + (cmp->get_y() + 1) * map_width_] ){
        n1->x( cmp->get_x() + 1 );
        n1->y( cmp->get_y() );
        neighbours_.push_back( n1 );
    }
}
A_Star_Path_Planner::~A_Star_Path_Planner()
{
    //dtor
}
