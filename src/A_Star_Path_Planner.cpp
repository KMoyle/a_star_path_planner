#include "../include/A_Star_Path_Planner.h"

A_Star_Path_Planner::A_Star_Path_Planner( MapCell start, MapCell goal,  std::vector<char> map )
{
    start_ = &start;
    goal_ = &goal;
    map_ = map;

    //init sizes of all a* vecs
    grid_.resize( map_height_ * map_width_ );
    parent_.resize( map_height_ * map_width_ );
    neighbour_ids_.resize( map_height_ * map_width_ );
    path_.reserve( map_height_ * map_width_ );
    open_set_.clear();
    closed_set_.clear();
    opened_.resize( map_height_ * map_width_ );
    fill(opened_.begin(), opened_.end(), false);
    closed_.resize( map_height_ * map_width_ );
    fill(closed_.begin(), closed_.end(), false);
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

        //finding cell with best f score
        MapCell *current = get_best_neighbour();
        float current_id_ = current->get_x() + current->get_y() * map_width_;



        //std::cout << "( " << current->get_x() << " , " << current->get_y() << " )" << std::endl;

        if ( current->get_x() == goal_->get_x() && current->get_y() == goal_->get_y() ){
            found_goal = true;
            std::cout << " FOUND PATH " << std::endl;
           compute_path( current );
            break;
        }
        //remove current point from opened set and place in closed set and set closed to true
        closed_set_.push_back( open_set_.front() );
        closed_[ current_id_ ] = true;
        map_[ current_id_ ] = 'C';
        if( DEBUG ) print_map();

        //get valid neighbours of current i.e. within bounds
        neighbour_ids_.clear();
        add_neighbours( current );

        //neighbour info
        if( DEBUG ) std::cout << "current ->  ( " <<  current->get_x() << " , " << current->get_y() << " )" <<  "\tg= " <<  g_[ current_id_ ] << "\tf= " <<  f_[ current_id_ ] << "\n" << std::endl;

        //loop through neighbours
        std::vector<unsigned int>::iterator itn;
        for( itn = neighbour_ids_.begin() ; itn != neighbour_ids_.end(); itn++ ){

            //firstly compute, cost = g(current) + distance(current,neighbour)
            float cost = g_[ current_id_ ] + return_g_score( current, *itn );



            //neighbour info
            if( DEBUG )std::cout << "neighbour ->  ( " <<  grid_[*itn].get_x() << " , " <<  grid_[*itn].get_y() << " )" <<  "\told g= " <<  g_[ *itn ] << "\tcost= " << cost << "\n" << std::endl;

            if( closed_[*itn] && cost < g_[ *itn ] ){

                     g_[ *itn ] = cost;
                     f_[ *itn ] = g_[ *itn ] + h_[ *itn ];
                    closed_[*itn] = false;
                    opened_[*itn] = true;
                    if( DEBUG )std::cout << "neighbours parent was ( " << parent_[ *itn ]->get_x() << " , " << parent_[ *itn ]->get_y() << " )" << std::endl;
                    parent_[ *itn ] = &grid_[current_id_];
                    if( DEBUG )std::cout << "neighbours parent is now ( " << parent_[ *itn ]->get_x() << " , " << parent_[ *itn ]->get_y() << " )" << std::endl;

            }else if ( opened_[*itn] && cost < g_[ *itn ] ){

                    g_[ *itn ] = cost;
                    f_[ *itn ] = g_[ *itn ] + h_[ *itn ];
                    if( DEBUG )std::cout << "neighbours parent was ( " << parent_[ *itn ]->get_x() << " , " << parent_[ *itn ]->get_y() << " )" << std::endl;
                    parent_[ *itn ] = &grid_[current_id_];
                    if( DEBUG )std::cout << "neighbours parent is now ( " << parent_[ *itn ]->get_x() << " , " << parent_[ *itn ]->get_y() << " )" << std::endl;
            }else if( !opened_[*itn] && !closed_[*itn] ){

                open_set_.push_back( &grid_[ *itn ] );
                opened_[ *itn  ] = true;
                g_[ *itn ] = cost;
                h_[ *itn ] = return_h_score( &grid_[ *itn ] );
                f_[ *itn ] = g_[ *itn ] + h_[ *itn ];
                map_[ *itn ] = 'O';

            }
        }

        map_[ current->get_x() + current->get_y() * map_width_ ] = 'V';
    }





    //ctor
}

MapCell *A_Star_Path_Planner::get_best_neighbour( ){

    MapCell *best;
    unsigned int current_id = 0;
    std::list<MapCell *>::iterator itf;
    std::list<MapCell *>::iterator best_itf;

    //iterating through the list to find the mapcell with the best f score
    for( itf = open_set_.begin(); itf != open_set_.end(); itf++){

        if ( itf == open_set_.begin() || ( ( f_[(*itf)->get_x() + (*itf)->get_y() * map_width_] ) < f_[current_id]  )  ){

            //std::cout << "f_new = " << ( f_[(*itf)->get_x() + (*itf)->get_y() * map_width_] ) << "\n" << "f_old= " <<  f_[current_id] << std::endl;
            current_id = (*itf)->get_x() + (*itf)->get_y() * map_width_;
            best = &grid_[current_id];
            best_itf = itf;
        }

    }

    open_set_.erase(best_itf);

    return best;

}


float A_Star_Path_Planner::return_g_score( MapCell *cmp, unsigned int neighbour_id ){

    //return abs( cmp->get_x() - grid_[neighbour_id].get_x() ) + abs( cmp->get_y() - grid_[neighbour_id].get_y());

    float dx = static_cast<float>( cmp->get_x() ) - static_cast<float>(grid_[neighbour_id].get_x());
    float dy = static_cast<float>( cmp->get_y() ) - static_cast<float>(grid_[neighbour_id].get_y());

    float g = ( dx * dx ) + ( dy * dy ) ;

            //std::cout << "g= " << g << std::endl;
    return  g;
}
float A_Star_Path_Planner::return_h_score( MapCell *mp ){

    //return abs( mp->get_x() - goal_->get_x() ) + abs( mp->get_y() - goal_->get_y() );
    float dx = static_cast<float>( mp->get_x() ) - static_cast<float>(goal_->get_x());
    float dy = static_cast<float>( mp->get_y() ) - static_cast<float>(goal_->get_y());

    float h = ( dx * dx ) + ( dy * dy ) ;
                //std::cout << "h= " << h << std::endl;
    return h;

}

void A_Star_Path_Planner::add_neighbours( MapCell *cmp ){
    // TODO add robustness for inf or obs for real maps
    unsigned int neighbour_id;
    unsigned int current_id_ = cmp->get_x() + cmp->get_y() * map_width_;


    for ( int y = -1; y < 2; y++){
        for ( int x = -1; x < 2; x++ ){

            if ( ( cmp->get_x() > 0 ) &&  cmp->get_y() > 0 && ( cmp->get_x() + 1 ) < map_width_ && ( cmp->get_y() + 1 ) < map_height_ && !closed_[ ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_] && map_[ ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_] != 'X'){
                neighbour_id = ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_;
                //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
                neighbour_ids_.push_back( neighbour_id );
                if ( !opened_[ neighbour_id] ){
                    parent_[ neighbour_id ] = &grid_[current_id_];
                }

            } else if ( ( cmp->get_x() == 0 ) && ( cmp->get_y() > 0 ) && !closed_[ ( cmp->get_y() + (y) ) * map_width_] && map_[ ( cmp->get_y() + (y) ) * map_width_] != 'X' ){
                neighbour_id =  ( cmp->get_y() + (y) ) * map_width_;
                //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
                neighbour_ids_.push_back( neighbour_id );

                if ( !opened_[ neighbour_id] ){
                    parent_[ neighbour_id ] = &grid_[current_id_];
                }

            } else if ( ( cmp->get_x() > 0 ) && ( cmp->get_y() == 0 ) && !closed_[ ( cmp->get_x() + (x) ) ] && map_[ ( cmp->get_x() + (x) ) ] != 'X'){
                neighbour_id = ( cmp->get_x() );
                //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
                neighbour_ids_.push_back( neighbour_id );

                if ( !opened_[ neighbour_id] ){
                    parent_[ neighbour_id ] = &grid_[current_id_];
                }

            } else if ( ( (cmp->get_x() + 1 ) ==  map_width_ ) && ( cmp->get_y() + 1 ) < map_height_ && !closed_[ ( cmp->get_x() ) +  ( cmp->get_y() + 1 ) * map_width_] && map_[ ( cmp->get_x() ) +  ( cmp->get_y() + 1 ) * map_width_ ] != 'X'){
                neighbour_id = ( ( cmp->get_x() ) +  ( cmp->get_y() + 1 ) * map_width_ );
                //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
                neighbour_ids_.push_back( neighbour_id );

                if ( !opened_[ neighbour_id] ){
                    parent_[ neighbour_id ] = &grid_[current_id_];
                }

            } else if ( ( (cmp->get_x() + 1 ) <  map_width_ ) && ( cmp->get_y() + 1 ) == map_height_ && !closed_[ ( cmp->get_x() + 1 ) +  ( cmp->get_y()  ) * map_width_] && map_[ ( cmp->get_x() + 1 ) +  ( cmp->get_y()  ) * map_width_ ] != 'X'){
                neighbour_id = ( ( cmp->get_x() + 1 ) +  ( cmp->get_y()  ) * map_width_ );
                //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
                neighbour_ids_.push_back( neighbour_id );

                if ( !opened_[ neighbour_id] ){
                    parent_[ neighbour_id ] = &grid_[current_id_];
                }

            } else if (  ( cmp->get_x() == 0 ) && ( cmp->get_y() == 0 ) && x >= 0 && y >= 0 && !closed_[ ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_] && map_[ ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_] != 'X'){
                neighbour_id = ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_;
                //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
                neighbour_ids_.push_back( neighbour_id );

                if ( !opened_[ neighbour_id] ){
                    parent_[ neighbour_id ] = &grid_[current_id_];
                }
            }

        }
    }

}



void A_Star_Path_Planner::compute_path( MapCell* current ){

    path_.push_back( current );

   // std::cout << "PATH (GOAL) -- >> " << "( " << current->get_x() << " , " <<current->get_y()<< " )" << std::endl;
    //loop through the parent vector and add all values to path
    while( current->get_x() != start_->get_x() &&  current->get_y() != start_->get_y() ){

        //std::cout << "next in path -- >> " << "( " << current->get_x() << " , " << current->get_y()<< " )" << std::endl;
        path_.push_back( parent_[ current->get_x() + current->get_y() * map_width_ ] );
        current = parent_[ current->get_x() + current->get_y() * map_width_ ];

    }
    std::reverse(path_.begin(),path_.end());

    //  USED TO PRINT & VALIDATE path_
    std::vector<MapCell *>::iterator itp;

    for ( itp = path_.begin() ; itp != path_.end() ; itp++){

            std::cout << "( " << (*itp)->get_x() << " , "<< (*itp)->get_y() << " )" << std::endl;
    }


}

void A_Star_Path_Planner::print_map( ){

    int map_counter = 0;

    std::vector<char>::iterator itm;

    for( itm = map_.begin(); itm != map_.end(); itm++ ){

        if ( map_counter % 10 == 0 ){
            std::cout << "\n";

        }
        std::cout <<"  " << *itm;

        map_counter++;

    }

    std::cout << "\n";



}


A_Star_Path_Planner::~A_Star_Path_Planner()
{
    //dtor
}
