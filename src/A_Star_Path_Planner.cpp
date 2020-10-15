#include "../include/A_Star_Path_Planner.h"

A_Star_Path_Planner::A_Star_Path_Planner( MapCell start, MapCell goal )
{
    start_ = &start;
    goal_ = &goal;

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


        std::cout << "( " << current->get_x() << " , " << current->get_y() << " )" << std::endl;

        if ( current->get_x() == goal_->get_x() && current->get_y() == goal_->get_y() ){
            found_goal = true;
            std::cout << " FOUND PATH " << std::endl;
            compute_path( current );
            break;
        }
        //remove current point from opened set and place in closed set and set closed to true
        closed_set_.push_back( open_set_.front() );
        closed_[ current->get_x() + current->get_y() * map_width_ ] = true;


        //get valid neighbours of current i.e. within bounds
        neighbour_ids_.clear();
        add_neighbours( current );

        //loop through neighbours
        std::vector<unsigned int>::iterator itn;
        for( itn = neighbour_ids_.begin() ; itn != neighbour_ids_.end(); itn++ ){

            //firstly compute, cost = g(current) + distance(current,neighbour)
            float cost = g_[ current->get_x() + current->get_y() * map_width_ ] + return_g_score( current, *itn );

            if ( opened_[*itn] && cost < g_[ *itn ] ){
                    //TODO remove neighbour from open list as new path is better
                continue;
            }else if( closed_[*itn] && cost < g_[ *itn ] ){
                    //TODO remove neighbour from closed list
                continue;

            }else if( !opened_[*itn] && !closed_[*itn] ){
                open_set_.push_back( &grid_[ *itn ] );
                opened_[ *itn  ] = true;
                g_[ *itn ] = cost;
                h_[ *itn ] = return_h_score( &grid_[ *itn ] );
                f_[ *itn ] = g_[ *itn ] + h_[ *itn ];

            }
        }
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

               // std::cout << "g= " << g << std::endl;
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
    MapCell parent = grid_[cmp->get_x() + cmp->get_y() * map_width_];


    //for x dir
    if ( cmp->get_x() > 0 && !closed_[ ( cmp->get_x()-1 ) + cmp->get_y() * map_width_] ){
        neighbour_id = ( cmp->get_x()-1 ) + cmp->get_y() * map_width_;
        //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
        neighbour_ids_.push_back( neighbour_id );
        parent_[ neighbour_id ] = &parent;
    }
    if ( cmp->get_x() + 1 < map_width_ && !closed_[ ( cmp->get_x()+1 ) + cmp->get_y() * map_width_] ){
        neighbour_id = ( cmp->get_x()+1 ) + cmp->get_y() * map_width_;
        //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
        neighbour_ids_.push_back( neighbour_id );
        parent_[ neighbour_id ] = &parent;
    }
    //for x & y
    if ( ( cmp->get_x() + 1 ) < map_width_ &&  ( cmp->get_y() + 1 ) < map_height_  && !closed_[ ( cmp->get_x()+1 ) + ( cmp->get_y()+1 ) * map_width_] ){
        neighbour_id = ( cmp->get_x()+1 ) +  ( cmp->get_y()+1 ) * map_width_;
        //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
        neighbour_ids_.push_back( neighbour_id );
        parent_[ neighbour_id ] = &parent;
    }
    if ( cmp->get_x() > 0 &&  cmp->get_y() > 0 && !closed_[ ( cmp->get_x()-1 ) + ( cmp->get_y()-1 ) * map_width_ ] ){
        neighbour_id = ( cmp->get_x()-1 ) + ( cmp->get_y()-1 ) * map_width_;
        //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
        neighbour_ids_.push_back( neighbour_id );
        parent_[ neighbour_id ] = &parent;
    }
    if ( cmp->get_x() > 0 &&  cmp->get_y() + 1 < map_height_  && !closed_[ ( cmp->get_x()-1 ) + ( cmp->get_y()+1 ) * map_width_] ){
        neighbour_id = ( cmp->get_x()-1 ) + ( cmp->get_y()+1 ) * map_width_;
        //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
        neighbour_ids_.push_back( neighbour_id );
        parent_[ neighbour_id ] = &parent;
    }
    if ( cmp->get_x() + 1 < map_width_ &&  cmp->get_y() > 0 && !closed_[ ( cmp->get_x()+1 ) + ( cmp->get_y()-1 ) * map_width_] ){
        neighbour_id = ( cmp->get_x()+1 ) + ( cmp->get_y()-1 ) * map_width_;
       // std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
        neighbour_ids_.push_back( neighbour_id );
        parent_[ neighbour_id ] = &parent;
    }
    //for y dir
    if ( cmp->get_y() > 0 && !closed_[ cmp->get_x() + ( cmp->get_y() - 1) * map_width_] ){
        neighbour_id =  cmp->get_x() + ( cmp->get_y() - 1 ) * map_width_;
        //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
        neighbour_ids_.push_back( neighbour_id );
        parent_[ neighbour_id ] = &parent;
    }
    if ( ( cmp->get_y() + 1 ) < map_height_ && !closed_[ cmp->get_x() + ( cmp->get_y() + 1 ) * map_width_] ){
        neighbour_id =  cmp->get_x() + ( cmp->get_y() + 1 ) * map_width_;
        //std::cout << "NEW Neighbour -- >> " << "( " << grid_[neighbour_id].get_x() << " , " <<grid_[neighbour_id].get_y()<< " )" << std::endl;
        neighbour_ids_.push_back( neighbour_id );
        parent_[ neighbour_id ] = &parent;
    }
}



void A_Star_Path_Planner::compute_path( MapCell* current ){

    MapCell *parent;
    path_.emplace_back( current );

    std::cout << "PATH (GOAL) -- >> " << "( " << current->get_x() << " , " <<current->get_y()<< " )" << std::endl;

    while( current->get_x() != start_->get_x() &&  current->get_y() != start_->get_y() ){

        parent = parent_[ current->get_x() + current->get_y() * map_width_ ];
        //std::cout << "next in path -- >> " << "( " << *parent->get_x() << " , " << *parent->get_y()<< " )" << std::endl;

        path_.emplace_back( parent );

        current = parent;

    }



}

A_Star_Path_Planner::~A_Star_Path_Planner()
{
    //dtor
}
