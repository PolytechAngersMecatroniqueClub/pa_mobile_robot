#ifndef PAEXPLORATIONMAP_H
#define PAEXPLORATIONMAP_H

#include "PaExplorationMap.h"
#include "PaPoint2Di.h"
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#define PA_FREE       0
#define PA_OBSTACLE   100
#define PA_FRONTIER   50
#define PA_UNKNOWN    -1

class PaExplorationMap {

    private:
        nav_msgs::OccupancyGrid _map;

        int _height;          // nb cells X
        int _width;           // nb cells Y
        float _resolution;    // m/cells (size of a cell in the world)
        float _mapPosX;    // m/cells (size of a cell in the world)
        float _mapPosY;    // m/cells (size of a cell in the world)
        
    public:
        PaExplorationMap();
        void initMap(std::string frame_name, int height, int width, float resolution);

        int         get_id_from_cell(int x, int y) const;
        int         get_id_from_cell(const PaPoint2Di& point) const;

        int         get_x_cell_from_world(double x) const;
        int         get_y_cell_from_world(double y) const;

        int         get_x_cell_from_id(int id) const;
        int         get_y_cell_from_id(int id) const;

        PaPoint2Di get_cell_from_world(double x, double y) const;
        double      get_x_world_from_cell(int cx) const;
        double      get_y_world_from_cell(int cy) const;

        PaPoint2Di get_point2Di_from_cell(int i) const;

        bool isWalkable(int x, int y) const;
        bool isWalkable(int id) const;

        bool isFrontier(const PaPoint2Di& point);

        PaPoint2Di getClosestFrontier_from_cell(const PaPoint2Di cell);

        int getHeight() const { return _height; }
        int getWidth() const { return _width; }
        int getSize() const { return _map.data.size(); }

        const nav_msgs::OccupancyGrid& getMap() const { return _map;}

        void obstacle_detected(double x_robot, double y_robot, double x_obs, double y_obs);
        void obstacle_detected(int x_cell_robot, int y_cell_robot, int x_cell_obs, int y_cell_obs);
        void add_val_at(int cx, int cy);
        void free_path(int x1, int y1, int x2, int y2);
        void rem_val_at(int cx, int cy);
        void setfrontier(int cx, int cy);
        void add_2_map(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose& pose);
        void add_2_map(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose);
};

#endif
