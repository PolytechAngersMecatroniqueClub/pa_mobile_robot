#include "PaExplorationMap.h"

#include<math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


PaExplorationMap::PaExplorationMap(){
}

void PaExplorationMap::initMap(std::string frame_name, int height, int width, float resolution){

    _height = height;
    _width = width;
    _resolution = resolution;
    _mapPosX = -(width*resolution)/2;
    _mapPosY = -(height*resolution)/2;

    // probability map initialization
    _map.header.frame_id = frame_name.c_str();
    _map.info.height = _height;         // cells Y
    _map.info.width = _width;           // cells X
    _map.info.resolution = _resolution; // m/cells;

    // The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.(geometry_msgs/Pose)
    _map.info.origin.position.x = _mapPosX;
    _map.info.origin.position.y = _mapPosY;
    _map.info.origin.position.z = 0;

    // we clean the map if needed
    _map.data.clear();
    _map.data.resize(_height*_width);

    // the probability cells are initialized with the _thresholdbelief value 
    for(int i=0; i<_height*_width; i++){
        _map.data[i] = PA_UNKNOWN;
    }

}

// int y = (int) (i / _nb_x);
// int x = i - y*_nb_x;

int PaExplorationMap::get_x_cell_from_id(int id) const{
    return (id - get_y_cell_from_id(id)*_width);
}

int PaExplorationMap::get_y_cell_from_id(int id) const{
    return (int) (id / _width);
}

int PaExplorationMap::get_id_from_cell(int x, int y) const{
    return x + y*_width;
}

int PaExplorationMap::get_id_from_cell(const PaPoint2Di& point) const{
    return get_id_from_cell(point.getX(), point.getY());
}

int PaExplorationMap::get_x_cell_from_world(double x) const{
    return (int)((x-_map.info.origin.position.x)/_resolution);
}

int PaExplorationMap::get_y_cell_from_world(double y) const{
    return (int)((y-_map.info.origin.position.y)/_resolution);
}

double PaExplorationMap::get_x_world_from_cell(int cx) const{
    return (cx+0.5) * _resolution + _mapPosX;
}

double PaExplorationMap::get_y_world_from_cell(int cy) const{
    return (cy+0.5) * _resolution + _mapPosY;
}


void PaExplorationMap::set_cell_to_unknown(const PaPoint2Di& point){
    _map.data[get_id_from_cell(point)] = PA_UNKNOWN;
}

void PaExplorationMap::set_cell_to_obstacle(const PaPoint2Di& point){
    _map.data[get_id_from_cell(point)] = PA_OBSTACLE;
}


PaPoint2Di PaExplorationMap::getClosestFrontier_from_cell(const PaPoint2Di cell){
    double dst = -1;
    PaPoint2Di closest = cell;
    PaPoint2Di tmp;
    for(int i=0; i<_map.data.size(); i++){
        if(_map.data[i] == PA_FRONTIER){
            tmp = get_point2Di_from_cell(i);
            if(dst < 0){
                dst = PaPoint2Di::dst(cell, tmp);
                closest = tmp;
            }else{
                if(dst > PaPoint2Di::dst(cell, tmp)){
                    closest = tmp;
                    dst = PaPoint2Di::dst(cell, tmp);
                }
            }
        }
    }
    return closest;
}

PaPoint2Di PaExplorationMap::get_point2Di_from_cell(int i) const{
    int y = (int) (i / _width);
    int x = i - y*_width;

    return PaPoint2Di(x, y);
}

PaPoint2Di PaExplorationMap::get_cell_from_world(double x, double y) const{
    return PaPoint2Di(get_x_cell_from_world(x), get_y_cell_from_world(y));
}


bool PaExplorationMap::isFrontier(const PaPoint2Di& point){
    return _map.data[get_id_from_cell(point)] == PA_FRONTIER;
}

bool PaExplorationMap::isWalkable(int x, int y) const{
    return isWalkable(get_id_from_cell(x, y));
}
bool PaExplorationMap::isWalkable(int id) const{
    if(_map.data[id] == PA_FREE || _map.data[id] == PA_FRONTIER){
        return true;
    }
    else return false;
}

// function to add a scan to the maps according to the pose
void PaExplorationMap::add_2_map(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose& pose){

    unsigned int i, size = scan.ranges.size();
    double angle = scan.angle_min;
    double obs_x, obs_y;
    std::vector<float>::const_iterator it_msr; //using an iterator for faster loops

    tf::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    double theta = yaw;

    // loop over the measurements
    for(it_msr=scan.ranges.begin(); it_msr!=scan.ranges.end(); ++it_msr){
        if(*it_msr == 0 || *it_msr >= INFINITY || *it_msr < 0.2f || *it_msr >= 100.0f){
            // I do not know why sometimes we go here... maybe it is because of bad scans...
        }
        else{
            // convert the scan into x y coordinates (according to the pose)
            obs_x = (*it_msr) * cos(angle + theta) + pose.position.x;
            obs_y = (*it_msr) * sin(angle + theta) + pose.position.y;

            // add the detected obstacle to the map
            // the pose coordinates are also needed to "free" the line between the pose and the obstacle
            obstacle_detected(pose.position.x, pose.position.y, obs_x, obs_y);
        }
        // needed to convert a scan into x,y coordinates
        angle += scan.angle_increment;
    }
}

void PaExplorationMap::add_2_map(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose){

    unsigned int i, size = scan.ranges.size();
    double angle = scan.angle_min;
    double obs_x, obs_y;
    std::vector<float>::const_iterator it_msr; //using an iterator for faster loops
    // loop over the measurements
    for(it_msr=scan.ranges.begin(); it_msr!=scan.ranges.end(); ++it_msr){
        if(*it_msr == 0 || *it_msr >= INFINITY || *it_msr < 0.2f || *it_msr >= 100.0f){
            // I do not know why sometimes we go here... maybe it is because of bad scans...
        }
        else{
            // convert the scan into x y coordinates (according to the pose)
            obs_x = (*it_msr) * cos(angle + pose.theta) + pose.x;
            obs_y = (*it_msr) * sin(angle + pose.theta) + pose.y;

            // add the detected obstacle to the map
            // the pose coordinates are also needed to "free" the line between the pose and the obstacle
            obstacle_detected(pose.x, pose.y, obs_x, obs_y);
        }
        // needed to convert a scan into x,y coordinates
        angle += scan.angle_increment;
    }
}

// to update the map when an obstacle is detected (inputs are world coordinates in m)
void PaExplorationMap::obstacle_detected(double x_robot, double y_robot, double x_obs, double y_obs){
    // first we convert the world coordinates into cell indexes
    int x_cell_r = get_x_cell_from_world(x_robot);
    int y_cell_r = get_y_cell_from_world(y_robot);
    int x_cell_o = get_x_cell_from_world(x_obs);
    int y_cell_o = get_y_cell_from_world(y_obs);

    // then we check if the obstacle and the robot belongs to the map
    if(x_cell_r >= 0 && y_cell_r >= 0 && x_cell_o >= 0 && y_cell_o >= 0 &&
       x_cell_r < _width && y_cell_r < _height && x_cell_o < _width && y_cell_o  < _height){
        // and we use the obstacle_detected function that needs cell coordinates
        obstacle_detected(x_cell_r, y_cell_r, x_cell_o, y_cell_o);
    } // if not, nothing to but but warn the user
    else ROS_ERROR("PaExplorationMap::obstacle_detected() - (%d %d) or (%d %d) is not a correct cell", x_cell_r, y_cell_r, x_cell_o, y_cell_o);
}


// to update the map when an obstacle is detected (inputs are cells coordinates)
void PaExplorationMap::obstacle_detected(int x_cell_robot, int y_cell_robot, int x_cell_obs, int y_cell_obs){
    // first we check if the cells belongs to the map
    if(x_cell_obs >= 0 && x_cell_obs < _width && y_cell_obs >= 0 && y_cell_obs < _height){
        // we add the obstacle in the probability map
        add_val_at(x_cell_obs, y_cell_obs);
        // from the robot to this obstacle, the belives of a free path is updated
        free_path(x_cell_robot, y_cell_robot, x_cell_obs, y_cell_obs);
    } // if not, nothing to but but warn the user
    else ROS_ERROR("PaExplorationMap::obstacle_detected() - (%d %d) or (%d %d) is not a correct cell", x_cell_robot, y_cell_robot, x_cell_obs, y_cell_obs);
}

// function to add a value to a given cell
void PaExplorationMap::add_val_at(int cx, int cy){
    // first we check if the cells belongs to the map
    if(cx >= 0 && cx < _width && cy >= 0 && cy < _height){
        // we compute the index of the cell according the cell coordinates
        int id = get_id_from_cell(cx, cy);
        // if the cell become a new obtabcle (it was under the thresholdBelief and now goes over), it is added to the cost map
        _map.data[id] = PA_OBSTACLE;
    }// if the cell does not belong to the map, nothing to do but warn the user
    else ROS_WARN("PaExplorationMap::add_val_at() - %d (%d) %d (%d) is not a correct cell", cx, _width, cy, _height);
}

void PaExplorationMap::free_path(int x1, int y1, int x2, int y2){

    bool steep = abs(y2-y1) > abs(x2-x1);

    if(steep) {
        std::swap<int>(x1,y1);
        std::swap<int>(x2,y2);
    }

    if(x1 > x2) {
        std::swap<int>(x1,x2);
        std::swap<int>(y1,y2);
    }

    int dx = x2 - x1;
    int dy = abs(y2 - y1);

    double error = dx/2;
    const int ystep = (y1 < y2) ? 1 : -1;

    int y = y1;
    const int max_x = x2;
    int x_tmp=0, y_tmp=0;

    for(int x = x1 ; x < max_x; x++){
        if(steep){
            x_tmp = y;
            y_tmp = x;
        }
        else{
            x_tmp = x;
            y_tmp = y;
        }

        rem_val_at(x_tmp, y_tmp);
        setfrontier(x_tmp, y_tmp);

        error -= dy;
        if(error < 0){
            y += ystep;
            error += dx;
        }
    }
}

// function to add a value to a given cell
void PaExplorationMap::rem_val_at(int cx, int cy){
    // first we check if the cells belongs to the map
    if(cx >= 0 && cx < _width && cy >= 0 && cy < _height){
        // we compute the index of the cell according the cell coordinates
        int id = get_id_from_cell(cx, cy);
        // if the obstacle disapeared (it goes under the threshold), we need to remove it from the costmap
        if(_map.data[id] == PA_UNKNOWN || _map.data[id] == PA_FRONTIER)
            _map.data[id] = PA_FREE;
    }// if the cell does not belong to the map, nothing to do but warn the user
    else ROS_WARN("PaExplorationMap::rem_val_at() - %d %d is not a correct cell", cx, cy);
}


void PaExplorationMap::setfrontier(int cx, int cy){
    // identify frontier
    int id;
    id = get_id_from_cell(cx, cy);
    if(_map.data[id] == PA_FREE){
        if(cx > 0){
            if(_map.data[get_id_from_cell(cx-1, cy)] == PA_UNKNOWN){
                _map.data[id] = PA_FRONTIER;
            }
        }
        if(cy > 0){
            if(_map.data[get_id_from_cell(cx, cy-1)] == PA_UNKNOWN){
                _map.data[id] = PA_FRONTIER;
            }
        }
        if(cx < _width-1){
            if(_map.data[get_id_from_cell(cx+1, cy)] == PA_UNKNOWN){
                _map.data[id] = PA_FRONTIER;
            }
        }
        if(cy < _height-1){
            if(_map.data[get_id_from_cell(cx, cy+1)] == PA_UNKNOWN){
                _map.data[id] = PA_FRONTIER;
            }
        }
    }
}
