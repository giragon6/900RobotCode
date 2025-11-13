
#include "rosbridge.hh"

FRC900Comm::FRC900Comm(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{
    ros::param::param<double>("~publish_rate", _publish_rate, 30.0);

    double ping_interval;
    ros::param::param<double>("~ping_interval", ping_interval, 0.5);

    double odom_timeout;
    ros::param::param<double>("~odom_timeout", odom_timeout, 0.1);
    _odom_timeout = ros::Duration(odom_timeout);

    ros::param::param<bool>("~publish_odom_tf", _publish_odom_tf, true);

    // update_compact_ids();

    camera_pub = nh.advertise<sensor_msgs::Image>("/zed_objdetect/left/image_rect_color", 50);
    camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/zed_objdetect/left/camera_info", 50);
    ping_send_pub = nh.advertise<std_msgs::Float64>("ping_send", 1);
    ping_pub = nh.advertise<std_msgs::Float64>("ping", 10);

    ping_return_sub = nh.subscribe<std_msgs::Float64>("ping_return", 1, &FRC900Comm::ping_return_callback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &FRC900Comm::odom_callback, this);

    _ping_timer = nh.createTimer(ros::Duration(ping_interval), &FRC900Comm::ping_timer_callback, this);

    ROS_INFO("frc900comm is ready");
}

// ---
// Sub callbacks
// ---

void FRC900Comm::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    _last_odom = *msg;
    _last_odom.header.stamp = ros::Time::now();
    if (_publish_odom_tf) {
        publish_odom_tf();
    }
}

void FRC900Comm::ping_return_callback(const std_msgs::Float64ConstPtr& msg)
{
    std_msgs::Float64 ping;
    ping.data = get_time() - msg->data;
    ping_pub.publish(ping);
}

// ---
// Timer callbacks
// ---

void FRC900Comm::ping_timer_callback(const ros::TimerEvent& event)
{
    std_msgs::Float64 msg;
    msg.data = get_time();
    ping_send_pub.publish(msg);
}

// ---
// Helpers
// ---

void FRC900Comm::update_compact_ids()
{
    XmlRpc::XmlRpcValue compact_ids_param;
    string key;
    if (!ros::param::search("tf_compact_ids", key)) {
        throw std::runtime_error("Failed to find tf_compact_ids parameter");
    }
    nh.getParam(key, compact_ids_param);

    if (compact_ids_param.getType() != XmlRpc::XmlRpcValue::Type::TypeArray ||
        compact_ids_param.size() == 0) {
        throw std::runtime_error("tf_compact_ids is the wrong type or size");
    }

    for (int index = 0; index < compact_ids_param.size(); index++) {
        if (compact_ids_param[index].getType() != XmlRpc::XmlRpcValue::TypeArray) {
            throw std::runtime_error("tf_compact_ids element is not a list");
        }
        vector<string> pair;
        if (compact_ids_param[index].size() != 2) {
            throw std::runtime_error("tf_compact_ids element is not size 2");
        }
        for (int pair_index = 0; pair_index < compact_ids_param[index].size(); pair_index++) {
            if (compact_ids_param[index][pair_index].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR_STREAM("tf_compact_ids element is not a string: " << compact_ids_param[index][pair_index]);
                throw std::runtime_error("tf_compact_ids element is not a string");
            }
            pair.emplace_back((string)compact_ids_param[index][pair_index]);
        }
        compact_ids.emplace_back(pair);
    }
}

double FRC900Comm::get_time() {
    return ros::Time::now().toSec();
}

void FRC900Comm::publish_compact_tf()
{
    geometry_msgs::TransformStamped transform;
    tf2_msgs::TFMessage compacted_tree;
    for (size_t index = 0; index < compact_ids.size(); index++)
    {
        string parent_frame_id = compact_ids[index][0];
        string child_frame_id = compact_ids[index][1];

        try {
            transform = _tf_buffer.lookupTransform(parent_frame_id, child_frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            continue;
        }

        compacted_tree.transforms.push_back(transform);
    }
    tf_compact_pub.publish(compacted_tree);
}

void FRC900Comm::publish_odom_tf()
{
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = _last_odom.header.stamp;
    tf_stamped.header.frame_id = _last_odom.header.frame_id;
    tf_stamped.child_frame_id = _last_odom.child_frame_id;
    tf_stamped.transform.translation.x = _last_odom.pose.pose.position.x;
    tf_stamped.transform.translation.y = _last_odom.pose.pose.position.y;
    tf_stamped.transform.translation.z = _last_odom.pose.pose.position.z;
    tf_stamped.transform.rotation.w = _last_odom.pose.pose.orientation.w;
    tf_stamped.transform.rotation.x = _last_odom.pose.pose.orientation.x;
    tf_stamped.transform.rotation.y = _last_odom.pose.pose.orientation.y;
    tf_stamped.transform.rotation.z = _last_odom.pose.pose.orientation.z;
    _tf_broadcaster.sendTransform(tf_stamped);
}

void FRC900Comm::check_odom()
{
    ros::Duration delta_time = ros::Time::now() - _last_odom.header.stamp;
    if (delta_time > _odom_timeout) {
        ROS_WARN_THROTTLE(1.0, "No odometry received for %f seconds", delta_time.toSec());
    }
}

int FRC900Comm::run()
{
    ros::Rate clock_rate(_publish_rate);  // Hz
    while (ros::ok())
    {
        clock_rate.sleep();
        publish_compact_tf();
        check_odom();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_comm");
    ros::NodeHandle nh;
    FRC900Comm node(&nh);
    return node.run();
}