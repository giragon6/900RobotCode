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
from behavior_actions.msg import RosPoseObservation, RosTargetObservation

class ObservationRepublisher:

    def __init__(self):
        self.pose_obs_pub = rospy.Publisher(f"/tagslam_pose_obs", RosPoseObservation, tcp_nodelay=True, queue_size=1)
        self.targ_obs_pub = rospy.Publisher(f"/tagslam_targ_obs", RosTargetObservation, tcp_nodelay=True, queue_size=1)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.tagslam_pose_sub = rospy.Subscriber("/tagslam/odom/body_frc_robot", Odometry, self.tag_detect_cb)
    
    def tagslam_cb(self, msg: Odometry):
        if (self.cam_info != None): 
            # need to convert Odometry -> RosPoseObservation and RosTargetObservation
            poseObs = RosPoseObservation()
            targObs = RosTargetObservation()
            rfarr.rawFiducials = []
            rfarr.header.stamp = msg.header.stamp
            rfarr.header.frame_id = msg.header.frame_id
            for tag in tags:
                raw_fiducial = RawFiducial()
                raw_fiducial.id = tag.id
                # the corners of the apriltag are undistorted, so we take the focal length and principal pt from the 
                # rectified projection matrix of the camera info
                #     [fx'  0  cx' Tx]
                # P = [ 0  fy' cy' Ty]
                #     [ 0   0   1   0]
                fx = self.cam_info.P[0]
                fy = self.cam_info.P[5]
                # principal point
                cx = self.cam_info.P[2]
                cy = self.cam_info.P[6]
                raw_fiducial.txnc = math.atan((tag.center.x - cx)/fx) * 180 / math.pi # have to convert to degrees
                raw_fiducial.tync = math.atan((tag.center.y - cy)/fy)
                cam_area = self.cam_info.height * self.cam_info.width
                # get area of apriltag from corners
                # using formula for clockwise oriented quadrilateral
                # A=0.5(x1y2+x2y3+x3y4+x4y1)-(x2y1+x3y2+x4y3+x1y4))
                c = tag.corners
                tag_area = 0.5*abs((c[0].x*c[1].y + c[1].x*c[2].y + c[2].x*c[3].y + c[3].x*c[0].y) - 
                                (c[0].y*c[1].x + c[1].y*c[2].x + c[2].y*c[3].x + c[3].y*c[0].x))
                raw_fiducial.ta = tag_area / cam_area
                
                # # get transform from camera to tag
                # cam_to_tag_tf = self.tf_buf.lookup_transform('cam0', f'tag_{tag.id}', rospy.Time(0))
                # # get dist (do we need this?)
                # x, y, z = cam_to_tag_tf.transform.translation.x, cam_to_tag_tf.transform.translation.y, cam_to_tag_tf.transform.translation.z
                # raw_fiducial.distToCamera = math.sqrt(x**2 + y**2 + z**2)
                raw_fiducial.distToCamera = 0 #TODO: FIX THIS
                
                #TODO: make this accurate...?
                raw_fiducial.ambiguity = 0
                rfarr.rawFiducials.append(raw_fiducial)
            self.raw_fid_pub.publish(rfarr)
        else:
            pass
            # rospy.logerr("Not forwarding tag detection data to NetworkTables: Camera info isn't valid!")

if __name__ == "__main__":
    rospy.init_node("nt_raw_fid_republisher")
    republisher = RawFiducialRepublisher()  
    rospy.spin()