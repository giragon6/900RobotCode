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
from behavior_actions.msg import RawFiducial, RawFiducialArrayStamped
from geometry_msgs.msg import TransformStamped

class OdomReceiver:

    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.cam_info_sub = rospy.Subscriber(f"/wpi_odom", Odometry, self.odom_cb)
    
    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation = msg.pose.pose.position
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)
        

if __name__ == "__main__":
    rospy.init_node("odom_receiver")
    republisher = OdomReceiver()  
    rospy.spin()