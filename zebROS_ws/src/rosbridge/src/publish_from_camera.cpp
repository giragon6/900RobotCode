#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

sensor_msgs::CameraInfo getCameraInfo() {
    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 480;
    camera_info.width = 640;
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = {0.04507368872502638, -0.19661785523443845, 0.001512452282124494, 0.004642557892851897, 0.0};
    camera_info.K = {772.0256907026344, 0.0, 342.98345181074393, 0.0, 776.8330401937692, 301.4710831302156, 0.0, 0.0, 1.0};
    camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.P = {771.5826416015625, 0.0, 344.5282774027328, 0.0, 0.0, 777.8756713867188, 301.43318857727536, 0.0, 0.0, 0.0, 1.0, 0.0};
    return camera_info;
}

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res) {
    ROS_INFO("SetCameraInfo service called");
    res.success = true;
    res.status_message = "Camera info set successfully.";
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "publish_from_camera");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_ERROR("Usage: publish_from_camera [camera ID]");
        return 1;
    }

    int camera_id = std::stoi(argv[1]);

    cv::VideoCapture cap(camera_id, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera with ID %d", camera_id);
        return 1;
    }

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/zed_objdetect/left/image_rect_color", 1);
    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/zed_objdetect/left/camera_info", 1);

    ros::ServiceServer service = nh.advertiseService("/zed_objdetect/left/set_camera_info", setCameraInfo);

    sensor_msgs::CameraInfo camera_info = getCameraInfo();

    ros::Rate rate(100);

    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            ROS_WARN("Empty frame captured");
            continue;
        }

        std_msgs::Header header;
        header.frame_id = "base_link";
        header.stamp = ros::Time::now();

        cv_bridge::CvImage cv_image(header, "bgr8", frame);
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);

        image_pub.publish(ros_image);

        camera_info.header = header;
        camera_info_pub.publish(camera_info);

        ros::spinOnce();
        rate.sleep();
    }

    cap.release();
    return 0;
}