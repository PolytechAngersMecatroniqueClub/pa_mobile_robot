#include <ros/ros.h>
#include <ros/param.h>

#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>

// to handle the mapping
#include "slam/PaSlam.h"

// this is for the file operations
#include <fstream>
#include <string.h>

// -------  SLAM parameters - Optimization Nelder and Mead -------

int max_cost_2add;      // max cost for a LiDAR scan to be added into the map
double max_dist_2add;      // max distance between the computed pose and the previous one for a LiDAR scan to be added
int max_notadded;       // max consecutive not added scans before stopping the map computation
int max_it_nandm;       // maximal number of Nelder and Mead iterations
int max_nochange_nandm; // maximal number of Nelder and Mead consecutive iterations with the same result

// -------  Topic parameters ------- TODO: comments!

int prob_map_pub_rate;
int cost_map_pub_rate;
int path_publishing_rate;
bool publish_lidar_pose_tf;
std::string scan_topic_name;
std::string path_topic_name;
std::string probmap_topic_name;
std::string costmap_topic_name;
int lidar_msg_queue;
int path_msg_queue;

// -------  Frame parameters -------

std::string base_link_name;
std::string lidar_link_name;
std::string map_link_name;

// -------  Map parameters -------

float resolution;
float height;
float width;
int cost_stamp_radius;
int max_belief;
int min_belief;
int add_belief;
int rem_belief;
int threshold_belief;

// -------  Initial pose parameters -------

float initial_x;     // Initial x position
float initial_y;     // Initial y position
float initial_theta; // Initial theta orientation

// variables of the node

int nb_iterations;        // counter to know when publishing the path
int cpt_not_added = 0;    // counter of not added scans
PaSlam* paSlam; // to handle the nelder and mead, and the maps
nav_msgs::Path path;            // the path (trajectory) of the robot
geometry_msgs::Pose2D pose;     // the current pose (according to the current LiDAR scan and current map)
bool map_started = false;           // flag to identify the first iteration
bool stop_mapping = false;          // flag to start/stop the mapping process
ros::Publisher path_pub;            // to publish the path

#include "init_functions/init_pa_slam_node.h"

// to transform a pitch/roll/yaw value to a quaternion
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw){
    geometry_msgs::Quaternion q;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x=(t0 * t3 * t4 - t1 * t2 * t5);
    q.y=(t0 * t2 * t5 + t1 * t3 * t4);
    q.z=(t1 * t2 * t4 - t0 * t3 * t5);
    return q;
}

// To save the path of the robot in order to be able to display it
void updatepath(){
    path.header.frame_id = base_link_name.c_str();
    // the 2D pose needs to be converted into a posestamped
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = base_link_name.c_str();
    pose_stamped.pose.position.x = pose.x;
    pose_stamped.pose.position.y = pose.y;
    pose_stamped.pose.orientation = toQuaternion(0.0f,0.0f,pose.theta);
    // adding the new pose to the path
    path.poses.push_back(pose_stamped);
}

// To publish the LiDAR transform based on the pose computed
void publisLiDARPoseTf(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.x, pose.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link_name.c_str(), lidar_link_name.c_str()));
}

// to add a LiDAR scan data into the map (main mapping function)
void addLidarScan(const sensor_msgs::LaserScan::ConstPtr& msg){
    // first it is needed to test if this is the first iteration
    if(! map_started){
        // if this is the first iteration (no map computed so far)
        ROS_INFO("pa_slam_base_node::Starting mapping!");
        // initialisation of the LiDAR pose
        pose.x = initial_x;
        pose.y = initial_y;
        pose.theta = initial_theta;
        paSlam->add_2_map(*msg, pose); // we directly add the first LiDAR data to the map as the origin of the map
        map_started = true; // we update the flag
        nb_iterations = 0;  // initialization of the number of iterations
        paSlam->publish_probability_map();  // we publish the first computed map
         // if enabled, the corresponding tf is published
        if(publish_lidar_pose_tf) publisLiDARPoseTf();
    }else if(!stop_mapping){
        // this is not the first iteration and the map has to be built (the SLAM is enable)
        geometry_msgs::Pose2D pose_tmp(pose);   // the current pose (according to the current LiDAR scan and current map)
        geometry_msgs::Pose2D old_pose;         // the last pose
        int watch_dog = 0; // initialiaztion of the watchdog (not to loop too long)
        do{
            int cpt_no_change = 0; // this counts the number of identical pose computed by the Nelder and Mead
            while(cpt_no_change < max_nochange_nandm){ // we are waiting for the nelder and mead to return the same value several times
                old_pose = pose_tmp; // we save the previous pose
                pose_tmp = paSlam->nelder_mead(*msg, pose_tmp); // we compute a new pose with nelder and mead optimization
                // we check if this new pose equals the previous one
                if(old_pose.x == pose_tmp.x && old_pose.y == pose_tmp.y && old_pose.theta == pose_tmp.theta){
                    cpt_no_change++;
                }else{
                    cpt_no_change=0;
                }
            }
            watch_dog++; // update the watch dog not to loop too long
        }while(paSlam->get_cost(*msg, pose_tmp) > max_cost_2add && watch_dog < max_it_nandm); // if we did too many iterations or the new pose has a valid cost

        double distance = (pose_tmp.x - pose.x)*(pose_tmp.x - pose.x) + (pose_tmp.y - pose.y)*(pose_tmp.y - pose.y);
        if(distance > max_dist_2add ){
            ROS_ERROR("pa_slam_base_node::scan not added! - distance: %2.2f", distance);
            cpt_not_added ++;
            if(cpt_not_added > max_notadded){
                stop_mapping = true;
            }
        }else if(watch_dog < max_it_nandm){
            // if we did not break the loop because of the watchdog
            // we add the new scan to the map
            pose = pose_tmp;
            paSlam->add_2_map(*msg, pose);
            // we update the path
            updatepath();
            // if enabled, the corresponding tf is published
            if(publish_lidar_pose_tf) publisLiDARPoseTf();
            cpt_not_added = 0;
        }else{
            // we break the loop because of the watch dog, meaning that the pose/lidar cost is to high...
            // we do not add the LiDAR scan, we stop the map building
            ROS_WARN("pa_slam_base_node::scan not added! - current cost: %2.2f", paSlam->get_cost(*msg, pose));
            cpt_not_added ++;
            if(cpt_not_added > max_notadded){
                stop_mapping = true;
            }
        }
        nb_iterations ++;

        // publish the different messages
        if(prob_map_pub_rate!=0 && nb_iterations%prob_map_pub_rate == 0){
            paSlam->publish_probability_map();
        }
        if(cost_map_pub_rate!=0 && nb_iterations%cost_map_pub_rate == 0){
            paSlam->publish_cost_map();
        }
        if(path_publishing_rate!=0 && nb_iterations%path_publishing_rate == 0){
            path_pub.publish(path);
        }
    }
}

// main function
int main(int argc, char **argv)
{
    // -------- Set up ROS. -------- 
    ros::init(argc, argv, "pa_slam_base_node");
    ros::NodeHandle node;

    init_pa_slam_node_from_param_server(node); // initialization of all the parameters from the launch file or the default value


    path_pub = node.advertise<nav_msgs::Path>(path_topic_name, path_msg_queue);
    ros::Subscriber sub_lidar = node.subscribe(scan_topic_name.c_str(), lidar_msg_queue, addLidarScan);


    ROS_INFO("pa_slam_base_node::Initialization of the map...");
    // creating the maps from the parameters
    paSlam = new PaSlam(costmap_topic_name, probmap_topic_name);
    paSlam->_map.init_maps((int)(height/resolution+resolution), (int)(width/resolution+resolution)+1, resolution, cost_stamp_radius);
    paSlam->_map._maxBelief = max_belief;
    paSlam->_map._minBelief = min_belief;
    paSlam->_map._addBelief = add_belief;
    paSlam->_map._remBelief = rem_belief;
    paSlam->_map._thresholdBelief = threshold_belief;
    paSlam->_map._probmap.header.frame_id = map_link_name.c_str();

    // init the map started flag
    map_started = false;

    // subscribe to the LiDAR scan topic
    ROS_INFO("pa_slam_base_node::Node started waiting for LiDAR scan to start mapping");

    // main ros loop
    while(ros::ok()) // nothing to do but wait for scan messages
    {   if(stop_mapping){
            ROS_ERROR("pa_slam_base_node:: Too many not considered scans, the map processing is stoped");
        }
        ros::spinOnce();
    }
    ROS_INFO("pa_slam_base_node:: End of the SLAM process...");

    return 0;
}

