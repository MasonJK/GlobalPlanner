#pragma once

// data tpye that carries (x,y) position
class Coordinate{
public:
    int x_;
    int y_;
    Coordinate(){x_ = 0; y_ = 0;}
    Coordinate(int x, int y){x_ = x; y_ = y;}
    bool operator==(Coordinate c){
        if((this->x_ == c.x_) && (this->y_ == c.y_))
            return true;
        else
            return false;
    }
    bool operator!=(Coordinate c){
        if((this->x_ == c.x_) && (this->y_ == c.y_))
            return false;
        else
            return true;
    }
};