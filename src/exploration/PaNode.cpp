#include "PaNode.h"

#include <math.h>


PaNode::PaNode(bool walkable, const PaPoint2Di& position, double s_cost, double h_cost, const PaPoint2Di& parent){
    _walkable = walkable;
    _position = position;
    _s_cost = s_cost;
    _h_cost = h_cost;
    _parent_position = parent;
}

bool PaNode::operator<(const PaNode& other) const{
    if(_f_cost == other._f_cost){
        return _s_cost < other._s_cost;
    }else{
        return _f_cost < other._f_cost;
    }
}


void PaNode::update_costs(double s_cost, double h_cost, const PaPoint2Di& parent_position){
    if(_s_cost > s_cost){
        _s_cost = s_cost;
        _parent_position = parent_position;
    }

    if(_h_cost > h_cost){
        _h_cost = h_cost;
    }

    _f_cost = _s_cost + _h_cost;
}
