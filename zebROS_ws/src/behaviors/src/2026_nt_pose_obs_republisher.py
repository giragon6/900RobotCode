#!/usr/bin/env python3
import rospy
import tf2_ros
import math
from nav_msgs.msg import Odometry
from frc_msgs.msg import MatchSpecificData
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from behavior_actions.srv import OverrideAllianceColor, OverrideAllianceColorRequest, OverrideAllianceColorResponse
from sensor_msgs.msg import CameraInfo
from gpu_apriltag_msgs.srv import SetAllowedTags, SetAllowedTagsRequest, SetAllowedTagsResponse
from apriltag_msgs.msg import ApriltagArrayStamped
from behavior_actions.msg import RosPoseObservation

class PoseObservationRepublisher:

    def __init__(self):
        self.pose_obs_pub = rospy.Publisher(f"/tagslam_bridge/pose_observations", RosPoseObservation, tcp_nodelay=True, queue_size=1)

        # self.tf_buf = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.tagslam_pose_sub = rospy.Subscriber("/tagslam/odom/body_frc_robot", Odometry, self.tagslam_cb)
    
    def tagslam_cb(self, msg: Odometry):
        # need to convert Odometry -> RosPoseObservation
        pose_obs = RosPoseObservation()
        pose_obs.timestamp = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9
        pose_obs.pose = msg.pose.pose
        pose_obs.ambiguity = 0 #TODO: make accurate
        pose_obs.tagCount = 1 #not really but idk
        pose_obs.averageTagDistance = 0.0 #TODO: ????
        self.pose_obs_pub.publish(pose_obs)

if __name__ == "__main__":
    rospy.init_node("nt_pose_obs_republisher")
    republisher = PoseObservationRepublisher()  
    rospy.spin()