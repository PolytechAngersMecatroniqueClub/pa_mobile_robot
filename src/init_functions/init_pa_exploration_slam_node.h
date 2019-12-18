
// function to initialize all the parameters according to default values or parameters defined in the launch file
void init_pa_exploration_slam_node_from_param_server(ros::NodeHandle& node){

    // -------  SLAM parameters - Optimization Nelder and Mead -------

    if(node.getParam("max_cost_2add", max_cost_2add)){
        ROS_INFO("pa_slam_base_node::max_cost_2add parameter (cost): %d", max_cost_2add);
    }else{
        max_cost_2add = 3000;
        ROS_WARN("pa_slam_base_node::Could not get the max_cost_2add parameter, default value (cost): %d", max_cost_2add);
    }
    if(node.getParam("max_dist_2add", max_dist_2add)){
        ROS_INFO("pa_slam_base_node::max_dist_2add parameter (meters): %2.2f", max_dist_2add);
    }else{
        max_dist_2add = 1.0;
        ROS_WARN("pa_slam_base_node::Could not get the max_dist_2add parameter, default value (meters): %2.2f", max_dist_2add);
    }
    if(node.getParam("max_notadded", max_notadded)){
        ROS_INFO("pa_slam_base_node::max_notadded parameter (nb iterations): %d", max_notadded);
    }else{
        max_notadded = 5;
        ROS_WARN("pa_slam_base_node::Could not get the max_notadded parameter, default value (nb iterations): %d", max_notadded);
    }
    if(node.getParam("max_nochange_nandm", max_nochange_nandm)){
        ROS_INFO("pa_slam_base_node::max_nochange_nandm parameter (nb iterations): %d", max_nochange_nandm);
    }else{
        max_nochange_nandm = 2;
        ROS_WARN("pa_slam_base_node::Could not get the max_nochange_nandm parameter, default value (nb iterations): %d", max_nochange_nandm);
    }
    if(node.getParam("max_it_nandm", max_it_nandm)){
        ROS_INFO("pa_slam_base_node::max_it_nandm parameter (nb iterations): %d", max_it_nandm);
    }else{
        max_it_nandm = 20;
        ROS_WARN("pa_slam_base_node::Could not get the max_it_nandm parameter, default value (nb iterations): %d", max_it_nandm);
    }

    // -------  Topic parameters -------

    prob_map_pub_rate = 100;
    if(node.getParam("prob_map_pub_rate", prob_map_pub_rate)){
        ROS_INFO("pa_slam_base_node::prob_map_pub_rate parameter (nb iteration): %d", prob_map_pub_rate);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the prob_map_pub_rate parameter, default value (nb iteration): %d", prob_map_pub_rate);
    }
    cost_map_pub_rate = 0;
    if(node.getParam("cost_map_pub_rate", cost_map_pub_rate)){
        ROS_INFO("pa_slam_base_node::cost_map_pub_rate parameter (nb iteration): %d", cost_map_pub_rate);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the cost_map_pub_rate parameter, default value (nb iteration): %d", cost_map_pub_rate);
    }
    path_publishing_rate = 10;
    if(node.getParam("path_publishing_rate", path_publishing_rate)){
        ROS_INFO("pa_slam_base_node::path_publishing_rate parameter (nb iteration): %d", path_publishing_rate);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the path_publishing_rate parameter, default value (nb iteration): %d", path_publishing_rate);
    }
    publish_lidar_pose_tf = true;
    if(node.getParam("publish_lidar_pose_tf", publish_lidar_pose_tf)){
        ROS_INFO("pa_slam_base_node::publish_lidar_pose_tf parameter (boolean): %d", publish_lidar_pose_tf);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the publish_lidar_pose_tf parameter, default value (boolean): %d", publish_lidar_pose_tf);
    }
    scan_topic_name = "/scan";
    if(node.getParam("scan_topic_name", scan_topic_name)){
        ROS_INFO("pa_slam_base_node::scan_topic_name (string): %s", scan_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the scan_topic_name parameter, default value (string): %s", scan_topic_name.c_str());
    }
    path_topic_name = "/paSlam/path";
    if(node.getParam("path_topic_name", path_topic_name)){
        ROS_INFO("pa_slam_base_node::path_topic_name (string): %s", path_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the path_topic_name parameter, default value (string): %s", path_topic_name.c_str());
    }
    probmap_topic_name = "/paSlam/probmap";
    if(node.getParam("probmap_topic_name", probmap_topic_name)){
        ROS_INFO("pa_slam_base_node::probmap_topic_name (string): %s", probmap_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the probmap_topic_name parameter, default value (string): %s", probmap_topic_name.c_str());
    }
    costmap_topic_name = "/paSlam/costmap";
    if(node.getParam("costmap_topic_name", costmap_topic_name)){
        ROS_INFO("pa_slam_base_node::costmap_topic_name (string): %s", costmap_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the costmap_topic_name parameter, default value (string): %s", costmap_topic_name.c_str());
    }
    lidar_msg_queue = 100;
    if(node.getParam("lidar_msg_queue", lidar_msg_queue)){
        ROS_INFO("pa_slam_base_node::lidar_msg_queue parameter (nb messages): %d", lidar_msg_queue);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the lidar_msg_queue parameter, default value (nb messages): %d", lidar_msg_queue);
    }
    path_msg_queue = 100;
    if(node.getParam("path_msg_queue", path_msg_queue)){
        ROS_INFO("pa_slam_base_node::path_msg_queue parameter (nb messages): %d", path_msg_queue);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the path_msg_queue parameter, default value (nb messages): %d", path_msg_queue);
    }

    // -------  Frame parameters -------

    base_link_name = "base_link_default";
    if(node.getParam("base_link_name", base_link_name)){
        ROS_INFO("pa_slam_base_node::base_link_name: %s", base_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the base_link_name parameter, default value: %s", base_link_name.c_str());
    }
    lidar_link_name = "laser_default";
    if(node.getParam("lidar_link_name", lidar_link_name)){
        ROS_INFO("pa_slam_base_node::lidar_link_name: %s", lidar_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the lidar_link_name parameter, default value: %s", lidar_link_name.c_str());
    }
    map_link_name = "map_default";
    if(node.getParam("map_link_name", map_link_name)){
        ROS_INFO("pa_slam_base_node::map_link_name parameter (string): %s", map_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the map_link_name parameter, default value (string): %s", map_link_name.c_str());
    }

    // -------  Map parameters -------

    resolution = 0.02;
    if(node.getParam("resolution", resolution)){
        ROS_INFO("pa_slam_base_node::resolution parameter (meters): %2.3f", resolution);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the resolution parameter, default value (meters): %2.3f", resolution);
    }
    height = 150;
    if(node.getParam("height", height)){
        ROS_INFO("pa_slam_base_node::height parameter (meters): %2.2f", height);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the height parameter, default value (meters): %2.2f", height);
    }
    width = 150;
    if(node.getParam("width", width)){
        ROS_INFO("pa_slam_base_node::width parameter (meters): %2.2f", width);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the width parameter, default value (meters): %2.2f", width);
    }
    cost_stamp_radius = 30;
    if(node.getParam("cost_stamp_radius", cost_stamp_radius)){
        ROS_INFO("pa_slam_base_node::cost_stamp_radius parameter (nb cells): %d", cost_stamp_radius);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the cost_stamp_radius parameter, default value (nb cells): %d", cost_stamp_radius);
    }
    max_belief = 100;
    if(node.getParam("max_belief", max_belief)){
        ROS_INFO("pa_slam_base_node::max_belief parameter (cost): %d", max_belief);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the max_belief parameter, default value (cost): %d", max_belief);
    }
    min_belief = 0;
    if(node.getParam("min_belief", min_belief)){
        ROS_INFO("pa_slam_base_node::min_belief parameter (cost): %d", min_belief);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the min_belief parameter, default value (cost): %d", min_belief);
    }
    add_belief = 25;
    if(node.getParam("add_belief", add_belief)){
        ROS_INFO("pa_slam_base_node::add_belief parameter (cost): %d", add_belief);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the add_belief parameter, default value (cost): %d", add_belief);
    }
    rem_belief = 10;
    if(node.getParam("rem_belief", rem_belief)){
        ROS_INFO("pa_slam_base_node::rem_belief parameter (cost): %d", rem_belief);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the rem_belief parameter, default value (cost): %d", rem_belief);
    }
    threshold_belief = 50;
    if(node.getParam("threshold_belief", threshold_belief)){
        ROS_INFO("pa_slam_base_node::threshold_belief parameter (cost): %d", threshold_belief);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the threshold_belief parameter, default value (cost): %d", threshold_belief);
    }

    // ------ Initialize pose parameters----

    initial_x = 0;
    if(node.getParam("initial_x", initial_x)){
        ROS_INFO("pa_slam_base_node::initial_x parameter (meters): %2.2f", initial_x);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the initial_x parameter, default value (meters): %2.2f", initial_x);
    }
    initial_y = 0;
    if(node.getParam("initial_y", initial_y)){
        ROS_INFO("pa_slam_base_node::initial_y parameter (meters): %2.2f", initial_y);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the initial_y parameter, default value (meters): %2.2f", initial_y);
    }
    initial_theta = 0;
    if(node.getParam("initial_theta", initial_theta)){
        ROS_INFO("pa_slam_base_node::initial_theta parameter (rad): %2.2f", initial_theta);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the initial_theta parameter, default value (rad): %2.2f", initial_theta);
    }


    // EXPLORATION!

    markers_frame_name = "markers_frame_name";
    if(node.getParam("markers_frame_name", markers_frame_name)){
        ROS_INFO("pa_slam_base_node::markers_frame_name parameter (string): %s", map_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the markers_frame_name parameter, default value (string): %s", markers_frame_name.c_str());
    }

    markers_namespace = "markers_namespace";
    if(node.getParam("markers_namespace", markers_namespace)){
        ROS_INFO("pa_slam_base_node::markers_namespace parameter (string): %s", map_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the markers_namespace parameter, default value (string): %s", markers_namespace.c_str());
    }

    linear_speed = 0.5;
    if(node.getParam("linear_speed", linear_speed)){
        ROS_INFO("pa_slam_base_node::linear_speed parameter (Twist: linear.x): %2.2f", linear_speed);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the linear_speed parameter, default value (Twist: linear.x): %2.2f", linear_speed);
    }

    angular_speed = 0.5;
    if(node.getParam("angular_speed", angular_speed)){
        ROS_INFO("pa_slam_base_node::angular_speed parameter (Twist: angular.z): %2.2f", angular_speed);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the angular_speed parameter, default value (Twist: angular.z): %2.2f", angular_speed);
    }

    explo_map_topic_name = "explo_map_topic_name";
    if(node.getParam("explo_map_topic_name", explo_map_topic_name)){
        ROS_INFO("pa_slam_base_node::explo_map_topic_name parameter (string): %s", explo_map_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the explo_map_topic_name parameter, default value (string): %s", explo_map_topic_name.c_str());
    }

    twist_topic_name = "twist_topic_name";
    if(node.getParam("twist_topic_name", twist_topic_name)){
        ROS_INFO("pa_slam_base_node::twist_topic_name parameter (string): %s", twist_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the twist_topic_name parameter, default value (string): %s", twist_topic_name.c_str());
    }

    astar_path_topic_name = "astar_path_topic_name";
    if(node.getParam("astar_path_topic_name", astar_path_topic_name)){
        ROS_INFO("pa_slam_base_node::astar_path_topic_name parameter (string): %s", astar_path_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the astar_path_topic_name parameter, default value (string): %s", astar_path_topic_name.c_str());
    }

    explo_map_frame_name = "explo_map_frame_name";
    if(node.getParam("explo_map_frame_name", explo_map_frame_name)){
        ROS_INFO("pa_slam_base_node::explo_map_frame_name parameter (string): %s", explo_map_frame_name.c_str());
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the explo_map_frame_name parameter, default value (string): %s", explo_map_frame_name.c_str());
    }

    publish_explomap_rate = 100;
    if(node.getParam("publish_explomap_rate", publish_explomap_rate)){
        ROS_INFO("pa_slam_base_node::publish_explomap_rate parameter (nb iteration): %d", publish_explomap_rate);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the publish_explomap_rate parameter, default value (nb iteration): %d", publish_explomap_rate);
    }

    update_twist_rate = 10;
    if(node.getParam("update_twist_rate", update_twist_rate)){
        ROS_INFO("pa_slam_base_node::update_twist_rate parameter (nb iteration): %d", update_twist_rate);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the update_twist_rate parameter, default value (nb iteration): %d", update_twist_rate);
    }

    explo_map_resolution = 0.3;
    if(node.getParam("explo_map_resolution", explo_map_resolution)){
        ROS_INFO("pa_slam_base_node::explo_map_resolution parameter (nb iteration): %2.3f", explo_map_resolution);
    }else{
        ROS_WARN("pa_slam_base_node::Could not get the explo_map_resolution parameter, default value (nb iteration): %2.3f", explo_map_resolution);
    }

}
