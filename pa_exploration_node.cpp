#include <ros/ros.h>
#include <ros/param.h>

#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>


// this is for the file operations
#include <fstream>
#include <string.h>

#include "src/exploration/PaAStar.h"


geometry_msgs::Pose     current_pose;

ros::Publisher probmap_pub; // to publish the probability map (occupancy grid)
ros::Publisher cmd_pub; // to publish the probability map (occupancy grid)
PaAStar* p_astar;
PaExplorationMap explo_map;
std::vector<PaPoint2Di> path;
geometry_msgs::Twist twist_cmd;
ros::Publisher path_vis_pub;
visualization_msgs::MarkerArray markers;

// variables for the launch file parameters

std::string markers_frame_name; // OK init
std::string markers_namespace; // OK init

double linear_speed; // OK init
double angular_speed; // OK init

std::string odom_topic_name; // OK init
std::string scan_topic_name; // OK init
std::string map_topic_name; // OK init
std::string twist_topic_name; // OK init
std::string path_topic_name; // OK init

std::string map_frame_name; // OK init

int publish_explomap_rate; // OK init
int update_twist_rate; // OK init

int map_height; // OK init
int map_width; // OK init
double map_resolution; // OK init


#include "src/init_functions/init_pa_exploration_node.h"


void publish_path(){
    
    if(markers.markers.size() < path.size()){
        markers.markers.resize(path.size());
    }
    for(int i=0; i<path.size(); i++){
        markers.markers[i].header.frame_id = markers_frame_name.c_str();
        markers.markers[i].header.stamp = ros::Time();
        markers.markers[i].ns = markers_namespace.c_str();
        markers.markers[i].id = i;
        markers.markers[i].type = visualization_msgs::Marker::SPHERE;
        markers.markers[i].action = visualization_msgs::Marker::ADD;
        markers.markers[i].pose.position.x = explo_map.get_x_world_from_cell(path[i].getX());
        markers.markers[i].pose.position.y = explo_map.get_y_world_from_cell(path[i].getY());
        markers.markers[i].pose.position.z = 0.1;
        markers.markers[i].pose.orientation.x = 0.0;
        markers.markers[i].pose.orientation.y = 0.0;
        markers.markers[i].pose.orientation.z = 0.0;
        markers.markers[i].pose.orientation.w = 0.0;
        markers.markers[i].scale.x = 0.1;
        markers.markers[i].scale.y = 0.1;
        markers.markers[i].scale.z = 0.1;
        markers.markers[i].color.a = 1.0; // Don't forget to set the alpha!
        markers.markers[i].color.r = 0.0;
        markers.markers[i].color.g = 1.0 - (float)i/path.size();
        markers.markers[i].color.b = 1.0;
    }
    for(int i=path.size(); i < markers.markers.size(); i++){
        markers.markers[i].action = 2;
    }
    path_vis_pub.publish( markers );
}


void newScan(const sensor_msgs::LaserScan::ConstPtr& msg){
    explo_map.add_2_map(*msg, current_pose);
    static int cpt = 0;
    cpt ++;

    if(publish_explomap_rate!=0){
        if(cpt %publish_explomap_rate == 0){
            probmap_pub.publish(explo_map.getMap());
        }
    }
}

// to add a LiDAR scan data into the map (main mapping function)
void newOdomPose(const nav_msgs::Odometry::ConstPtr& msg){
    current_pose = msg->pose.pose;
    static int cpt = 0;
    cpt ++;

    if(path.size() > 0){
        if( PaPoint2Di( explo_map.get_x_cell_from_world(current_pose.position.x),
                        explo_map.get_y_cell_from_world(current_pose.position.y)) == path[path.size()-1]){
                path.erase(path.end());
                ROS_INFO("item from path erased");
                publish_path();

                if(!explo_map.isFrontier(path[path.size()-1])){
                    ROS_INFO("target no longer a frontier");
                    path.clear();
                }else{
                    // we update the path just in case...
                    
                    PaPoint2Di robot = explo_map.get_cell_from_world(current_pose.position.x, current_pose.position.y);
                    path = p_astar->find_path(robot, path[path.size()-1]);
                }
                cpt = 0;
        }
    }
    if(cpt%update_twist_rate ==0){
        publish_path();
        twist_cmd.linear.x  = 0;
        twist_cmd.angular.z = 0;
        if(path.size() > 0){
            // get the pose orientation
            tf::Quaternion q(
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w);

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            //double theta = yaw;
            
            double alpha = atan2(-explo_map.get_y_world_from_cell(path[path.size()-1].getY()) + current_pose.position.y,
                                  explo_map.get_x_world_from_cell(path[path.size()-1].getX()) - current_pose.position.x) + yaw;

            double offset = 10*3.14159/180.0;

            twist_cmd.linear.x  = 0; //SPEED;
            twist_cmd.angular.z = 0;

            if(alpha < 0){
                if (-3.14159 <= alpha && alpha < -offset){
                    twist_cmd.linear.x  = 0.0;
                    twist_cmd.angular.z = angular_speed;
                }else if(-2*3.14159 + offset < alpha && alpha < -3.14159){
                    twist_cmd.linear.x  = 0.0;
                    twist_cmd.angular.z = -angular_speed;
                }else{
                    twist_cmd.linear.x  = linear_speed;
                }
            }else{
                if(3.14159 <= alpha && alpha < 2*3.14159-offset){
                    twist_cmd.linear.x  = 0.0;
                    twist_cmd.angular.z = angular_speed;
                }else if(offset < alpha && alpha < 3.14159){
                    twist_cmd.linear.x  = 0.0;
                    twist_cmd.angular.z = -angular_speed;
                }else{
                    twist_cmd.linear.x  = linear_speed;
                }
            }
        }else{
            

            cpt = -1;
            ROS_INFO("no target, should search for one");

            PaPoint2Di robot = explo_map.get_cell_from_world(current_pose.position.x, current_pose.position.y);
            PaPoint2Di target = explo_map.getClosestFrontier_from_cell(robot);
            if(robot!=target){
                path = p_astar->find_path(robot, target);
            }else{
                ROS_INFO("No more to explore...");
            }
        }
        cmd_pub.publish(twist_cmd);
    }
}


void stopnode(){
    delete p_astar;
    twist_cmd.linear.x  = 0;
    twist_cmd.angular.z = 0;
    cmd_pub.publish(twist_cmd);
    ROS_INFO("stopping pa_exploration_node");
}
// main function
int main(int argc, char **argv)
{
    // atexit(stopnode);
    // Set up ROS.
    ros::init(argc, argv, "pa_exploration_node",1);
    ros::NodeHandle node("~"); // to get the private parameters

    init_pa_exploration_node_from_param_server(node);

    // suscribers to the Odometry data
    //odom_topic_name = "/ground_truth/state";
    ros::Subscriber sub_odom = node.subscribe(odom_topic_name.c_str(), 1000, newOdomPose);

    // suscribers to the Odometry data
    //scan_topic_name = "/romulux/laser_scan/scan";
    ros::Subscriber sub_scan = node.subscribe(scan_topic_name.c_str(), 1000, newScan);

 
    probmap_pub = node.advertise<nav_msgs::OccupancyGrid>(map_topic_name.c_str(), 1000);

    //cmd_pub = node.advertise<geometry_msgs::Twist>("/romulux/base_controller/cmd_vel", 1000);

    cmd_pub = node.advertise<geometry_msgs::Twist>(twist_topic_name.c_str(), 1000);

    path_vis_pub = node.advertise<visualization_msgs::MarkerArray>( path_topic_name.c_str(), 10 );


    explo_map.initMap(map_frame_name, map_height, map_width, map_resolution);
    p_astar = new PaAStar(&explo_map);

    // init_parameters(node); // initialization of the parameters
   // HERE!!!
    // main ros loop
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

