#pragma once

#include "ros/ros.h"

using namespace std;

class FRC900Comm
{
    /**
     * This node communicates with the RoboRIO over specific ROS topics (assumes ROS is running
     * on the RIO).
     * 
     * Published topics to the RIO
     *  /tf_compact - republishes specific transforms from /tf (configurable)
     *  /tj2/ping_send - for checking round trip latency and clock syncing
     * 
     * Subscribed topics from the RIO
     *  /tj2/odom - wheel odometry information
     *  /tj2/ping_return - the same data that gets published to /tj2/ping_send
     * 
     * Topics the RIO subscribes to but aren't published by this node
     *  /tj2/cmd_vel - twist_mux
     *  /tj2/detections - tj2_yolo
     *  /tj2/waypoints - tj2_waypoints
     *  /tj2/zones_info - tj2_zones
     * 
     * Topics the RIO publishes to but aren't handled by this node
     *  /initialpose - for AMCL
     *  /tj2/match - for tj2_match_watcher
     *  /tj2/<joint name> - for tj2_description
     *  /tj2/nogo_zones - for tj2_zones
     *  /tj2/record - for tj2_match_watcher
     * 
     * Other topics:
     *  /tj2/ping - the delay between the last ping_send and ping_return message
    */
private:
    ros::NodeHandle nh;  // ROS node handle
    
    // Parameters
    vector<vector<string>> compact_ids;
    double _publish_rate;
    bool _publish_odom_tf;
    
    // Members
    ros::Timer _ping_timer;
    ros::Duration _odom_timeout;
    nav_msgs::Odometry _last_odom;
    
    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher tf_compact_pub;
    ros::Publisher ping_send_pub;
    ros::Publisher ping_pub;
    
    // Subscribers
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    ros::Subscriber odom_sub;
    ros::Subscriber ping_return_sub;
    
    // Sub callbacks
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void ping_return_callback(const std_msgs::Float64ConstPtr& msg);

    // Timer callbacks
    void ping_timer_callback(const ros::TimerEvent& event);

    // Helpers
    void publish_compact_tf();
    void publish_odom_tf();
    void check_odom();
    double get_time();
    void update_compact_ids();

public:
    FRC900Comm(ros::NodeHandle* nodehandle);
    int run();
};
