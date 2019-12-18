
// function to initialize all the parameters according to default values or parameters defined in the launch file
void init_pa_exploration_node_from_param_server(ros::NodeHandle& node){

    // ------- PATH  ------- 

    markers_frame_name = "/markers_frame_name";
    if(node.getParam("markers_frame_name", markers_frame_name)){
        ROS_INFO("pa_exploration_node::markers_frame_name (string): %s", markers_frame_name.c_str());
    }else{
        ROS_WARN("pa_exploration_node::Could not get the markers_frame_name parameter, default value (string): %s", markers_frame_name.c_str());
    }

    path_topic_name = "/path_topic_name";
    if(node.getParam("path_topic_name", path_topic_name)){
        ROS_INFO("pa_exploration_node::path_topic_name (string): %s", path_topic_name.c_str());
    }else{
        ROS_WARN("pa_exploration_node::Could not get the path_topic_name parameter, default value (string): %s", path_topic_name.c_str());
    }

    markers_namespace = "/markers_namespace";
    if(node.getParam("markers_namespace", markers_namespace)){
        ROS_INFO("pa_exploration_node::markers_namespace (string): %s", markers_namespace.c_str());
    }else{
        ROS_WARN("pa_exploration_node::Could not get the markers_namespace parameter, default value (string): %s", markers_namespace.c_str());
    }

    // ------- MAP  ------- 

    map_frame_name = "/map_frame_name";
    if(node.getParam("map_frame_name", map_frame_name)){
        ROS_INFO("pa_exploration_node::map_frame_name (string): %s", map_frame_name.c_str());
    }else{
        ROS_WARN("pa_exploration_node::Could not get the map_frame_name parameter, default value (string): %s", map_frame_name.c_str());
    }

    map_topic_name = "/map_topic_name";
    if(node.getParam("map_topic_name", map_topic_name)){
        ROS_INFO("pa_exploration_node::map_topic_name (string): %s", map_topic_name.c_str());
    }else{
        ROS_WARN("pa_exploration_node::Could not get the map_topic_name parameter, default value (string): %s", map_topic_name.c_str());
    }

    publish_explomap_rate = 20;
    // note: 0 pour le pas publier!
    if(node.getParam("publish_explomap_rate", publish_explomap_rate)){
        ROS_INFO("pa_exploration_node::publish_explomap_rate parameter (int: nb of received LiDAR scan defore sending the map): %d", publish_explomap_rate);
    }else{
        ROS_WARN("pa_exploration_node::Could not get the publish_explomap_rate parameter, default value (int: nb of received LiDAR scan defore sending the map): %d", publish_explomap_rate);
    }

    map_height = 200;
    // note: 0 pour le pas publier!
    if(node.getParam("map_height", map_height)){
        ROS_INFO("pa_exploration_node::map_height parameter (int: nb cells - y): %d", map_height);
    }else{
        ROS_WARN("pa_exploration_node::Could not get the map_height parameter, default value (int: nb cells - y): %d", map_height);
    }

    map_width = 200;
    // note: 0 pour le pas publier!
    if(node.getParam("map_width", map_width)){
        ROS_INFO("pa_exploration_node::map_width parameter (int: nb cells - x): %d", map_width);
    }else{
        ROS_WARN("pa_exploration_node::Could not get the map_width parameter, default value (int: nb cells - x): %d", map_width);
    }

    map_resolution = 0.5;
    if(node.getParam("map_resolution", map_resolution)){
        ROS_INFO("pa_exploration_node::map_resolution parameter (double: meter): %2.2f", map_resolution);
    }else{
        ROS_WARN("pa_exploration_node::Could not get the map_resolution parameter, default value (double: meter): %2.2f", map_resolution);
    }

    // ------- LIDAR  ------- 

    scan_topic_name = "/scan_topic_name";
    if(node.getParam("scan_topic_name", scan_topic_name)){
        ROS_INFO("pa_exploration_node::scan_topic_name (string): %s", scan_topic_name.c_str());
    }else{
        ROS_WARN("pa_exploration_node::Could not get the scan_topic_name parameter, default value (string): %s", scan_topic_name.c_str());
    }

    // ------- ODOM  ------- 

    odom_topic_name = "/odom_topic_name";
    if(node.getParam("odom_topic_name", odom_topic_name)){
        ROS_INFO("pa_exploration_node::odom_topic_name (string): %s", odom_topic_name.c_str());
    }else{
        ROS_WARN("pa_exploration_node::Could not get the odom_topic_name parameter, default value (string): %s", odom_topic_name.c_str());
    }

    // ------- TWIST  ------- 

    twist_topic_name = "/twist_topic_name";
    if(node.getParam("twist_topic_name", twist_topic_name)){
        ROS_INFO("pa_exploration_node::twist_topic_name (string): %s", twist_topic_name.c_str());
    }else{
        ROS_WARN("pa_exploration_node::Could not get the twist_topic_name parameter, default value (string): %s", twist_topic_name.c_str());
    }

    linear_speed = 0.5;
    if(node.getParam("linear_speed", linear_speed)){
        ROS_INFO("pa_exploration_node::linear_speed parameter (double: twist.linear.x): %2.2f", linear_speed);
    }else{
        ROS_WARN("pa_exploration_node::Could not get the linear_speed parameter, default value (double: twist.linear.x): %2.2f", linear_speed);
    }

    angular_speed = 1.0;
    if(node.getParam("angular_speed", angular_speed)){
        ROS_INFO("pa_exploration_node::angular_speed parameter (double: twist.angular.z): %2.2f", angular_speed);
    }else{
        ROS_WARN("pa_exploration_node::Could not get the angular_speed parameter, default value (double: twist.angular.z): %2.2f", angular_speed);
    }

    update_twist_rate = 20;
    if(node.getParam("update_twist_rate", update_twist_rate)){
        ROS_INFO("pa_exploration_node::update_twist_rate parameter (int: nb pose to update twist): %d", update_twist_rate);
    }else{
        ROS_WARN("pa_exploration_node::Could not get the update_twist_rate parameter, default value (int: nb pose to update twist): %d", update_twist_rate);
    }

}
