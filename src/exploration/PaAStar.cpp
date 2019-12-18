#include "PaAStar.h"

#include<limits>
#include<math.h>

#include <ros/ros.h> // pour les tests

PaAStar::PaAStar(const PaExplorationMap* pmap){
    _pmap = pmap;
    
    _nodes.resize(_pmap->getSize());
    _reset();
}

void PaAStar::_reset(){
    for(int i=0; i<_nodes.size(); i++){
        // cx+cy*width = i
        // cx = i-cy*width
        // cy = (i - cx) / width

        int x = _pmap->get_x_cell_from_id(i);
        int y = _pmap->get_y_cell_from_id(i);

        _nodes[i].setPosition(x, y);
        _nodes[i].setScost(std::numeric_limits<double>::infinity());
        _nodes[i].setHcost(std::numeric_limits<double>::infinity());
        _nodes[i].setFcost(std::numeric_limits<double>::infinity());
    
        _nodes[i].setOpened(false);
        _nodes[i].setClosed(false);

        if(_pmap->isWalkable(i)){
            _nodes[i].setWalkable(true);
        }else{
            _nodes[i].setWalkable(false);
        }
    }
    _closed.clear();
    _opened.clear();
}


int PaAStar::heuristic(const PaPoint2Di& position, const PaPoint2Di& target){
    return (int)(sqrt(pow(position.getX() - target.getX(), 2) + pow(position.getY() - target.getY(), 2))*10);
}

std::vector<PaPoint2Di> PaAStar::find_path(const PaPoint2Di& start, const PaPoint2Di& target){
    _reset();

    int id = _pmap->get_id_from_cell(start.getX(), start.getY());
    _nodes[id].update_costs(0, heuristic(start, target), start);
    _nodes[id].setParentPosition(_nodes[id].getPosition());
    _opened.push_back(start);

    bool flag_found_path = false;

    while(_opened.size() > 0){
        int imin = 0;

        for(int i=1; i<_opened.size(); i++){
            if(_nodes[imin] < _nodes[i]){
                imin = i;
            }
        }
        PaPoint2Di current_pose = _opened[imin];
        _opened.erase(_opened.begin() + imin);

        if(current_pose == target){
            flag_found_path = true;
            _nodes[_pmap->get_id_from_cell(current_pose)].setClosed(true);
            _closed.push_back(current_pose);
            break;
        }

        update_node(current_pose, -1, -1, target);
        update_node(current_pose,  0, -1, target);
        update_node(current_pose,  1, -1, target);
        update_node(current_pose, -1,  0, target);
        update_node(current_pose,  1,  0, target);
        update_node(current_pose, -1,  1, target);
        update_node(current_pose,  0,  1, target);
        update_node(current_pose,  1,  1, target);

        _nodes[_pmap->get_id_from_cell(current_pose)].setClosed(true);
        _closed.push_back(current_pose);
    }
    std::vector<PaPoint2Di> path_PaPoint2Di(0);
    if(flag_found_path == true){
        ROS_INFO("PaAStar::target found!");
        PaPoint2Di current_pose = _closed[_closed.size()-1];
        while(current_pose != start){
            path_PaPoint2Di.push_back(current_pose);
            current_pose = _nodes[_pmap->get_id_from_cell(current_pose)].getParentPosition();
        }
    }else{
        ROS_WARN("PaAStar::target not found!");
    }
    return path_PaPoint2Di;
}

void PaAStar::update_node(const PaPoint2Di& pos_parent, int dx, int dy, const PaPoint2Di& target){
    int x = pos_parent.getX();
    int y = pos_parent.getY();
    if ((0 <= x + dx) && (x + dx < _pmap->getWidth()) && (0 <= y + dy) && (y + dy < _pmap->getHeight())){
        int id = _pmap->get_id_from_cell(x+dx, y+dy);
        if(_nodes[id].isWalkable() == true){
            int dst = 14;
            if(dx == 0 || dy == 0){
                dst = 10;
            }
            _nodes[id].update_costs(_nodes[_pmap->get_id_from_cell(x, y)].getScost() + dst,
                                    heuristic(_nodes[id].getPosition(), target),
                                    pos_parent);

            if(!_nodes[id].isOpened() && !_nodes[id].isClosed()){
                    _opened.push_back(_nodes[id].getPosition());
                    _nodes[id].setOpened(true);
            }
        }
    }
}




