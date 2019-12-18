#include <ros/ros.h>
#include <ros/param.h>

#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// this is for the file operations
#include <fstream>
#include <string.h>

int path_publishing_rate;          // number of iteration (processed LiDAR data) before publishing the path

std::string base_link_name;     // Base link name (frame)
std::string path_topic_name;     // Base link name (frame)
nav_msgs::Path path;            // the path (trajectory) of the robot
geometry_msgs::Pose2D pose;     // the current pose (according to the current LiDAR scan and current map)

ros::Publisher path_pub;    // to publish the path

unsigned int pose_counter = 0;

// function to initialize all the parameters according to default values or parameters defined in the launch file
void init_parameters(ros::NodeHandle& node){

    // ---------- GLOBAL VARIABLES ---------- 

     // Base Link name (frame)
    base_link_name = "romulux_odom";
    if(node.getParam("base_link_name", base_link_name)){
        ROS_INFO("odom2path_node::base link name: %s", base_link_name.c_str());
    }else{
        ROS_WARN("odom2path_node::Could not get the base link name parameter, default value: %s", base_link_name.c_str());
    }

    // to publish the path of the robot
    std::string path_topic_name = "/odom2path/path";
    if(node.getParam("path_topic_name", path_topic_name)){
        ROS_INFO("odom2path_node::path topic name: %s", path_topic_name.c_str());
    }else{
        ROS_WARN("odom2path_node::Could not get the path topic name parameter, default value: %s", path_topic_name.c_str());
    }
    path_pub = node.advertise<nav_msgs::Path>(path_topic_name, 1000);

    // number of iteration (processed LiDAR data) before publishing the path
    path_publishing_rate = 10;
    if(node.getParam("path_publishing_rate", path_publishing_rate)){
        ROS_INFO("odom2path_node::path_publishing_rate parameter (nb iteration): %d", path_publishing_rate);
    }else{
        ROS_WARN("odom2path_node::Could not get the path_publishing_rate parameter, default value (nb iteration): %d", path_publishing_rate);
    }

}


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
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}

// To save the path of the robot in order to be able to display it
void updatepath(geometry_msgs::Pose pose){
    path.header.frame_id = base_link_name.c_str();
    // the 2D pose needs to be converted into a posestamped
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = base_link_name.c_str();
    pose_stamped.pose.position.x = pose.position.x;
    pose_stamped.pose.position.y = pose.position.y;
    pose_stamped.pose.orientation = toQuaternion(0.0f,0.0f,pose.orientation.z);
    // adding the new pose to the path
    path.poses.push_back(pose_stamped);
}

// to add a LiDAR scan data into the map (main mapping function)
void addOdomPose(const nav_msgs::Odometry::ConstPtr& msg){
    // first it is needed to test if it is the first iteration
    updatepath(msg->pose.pose);
    pose_counter++;
    if(pose_counter >= path_publishing_rate){
        pose_counter = 0;
        path_pub.publish(path);
    }
    
}

// main function
int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "test",1);
    ros::NodeHandle node("~"); // to get the private parameters

    // suscribers to the Odometry data
    std::string odom_topic_name = "/ground_truth/state";
    if(node.getParam("odom_topic_name", odom_topic_name)){
        ROS_INFO("odom2path_node::odom topic name: %s", odom_topic_name.c_str());
    }else{
        ROS_WARN("odom2path_node::Could not get the odom topic name parameter, default value: %s", odom_topic_name.c_str());
    }
    ros::Subscriber sub_odom = node.subscribe(odom_topic_name.c_str(), 1000, addOdomPose);

    ros::Rate loop_rate(10); // frequency of the ros loop (in Hz)

    init_parameters(node); // initialization of the parameters

    // main ros loop
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

