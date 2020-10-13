#ifndef MAP_CELL_H
#define MAP_CELL_H


class MapCell
{
    public:
        MapCell(){
            x_=0;
            y_=0;
        };

        MapCell( unsigned int x, unsigned int y ){
            x_=x;
            y_=y;
        };

        void x( unsigned int x ){
            x_=x;
        };

        void y( unsigned int y ){
            y_=y;
        };

        unsigned int get_x(){
            return x_;
        };

        unsigned int get_y(){
            return y_;
        };



        virtual ~MapCell(){};

    private:

        unsigned int x_;
        unsigned int y_;
};

#endif // A_STAR_PATH_PLANNER_H

