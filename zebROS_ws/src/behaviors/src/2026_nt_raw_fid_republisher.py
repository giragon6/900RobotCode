#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from frc_msgs.msg import MatchSpecificData
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from behavior_actions.srv import OverrideAllianceColor, OverrideAllianceColorRequest, OverrideAllianceColorResponse
from sensor_msgs.msg import CameraInfo
from gpu_apriltag_msgs.srv import SetAllowedTags, SetAllowedTagsRequest, SetAllowedTagsResponse
from apriltag_msgs.msg import ApriltagArrayStamped
from behavior_actions.msg import RawFiducial, RawFiducialArrayStamped

# FIELD_LENGTH = 17.55
# RED_TAGS = [6,7,8,9,10,11]
# BLUE_TAGS = [17,18,19,20,21,22]

class RawFiducialRepublisher:
    # tagslam_alliance = MatchSpecificData.ALLIANCE_COLOR_UNKNOWN
    # tagslam_last_x = None
    # tagslam_last_time = None
    # robot_enabled = True
    # manual_override_enabled = False
    # manual_alliance = MatchSpecificData.ALLIANCE_COLOR_UNKNOWN
    # red_ignored = True
    # blue_ignored = True
    
    cam_info = None

    def __init__(self):
        # self.tagslam_sub = rospy.Subscriber("/tagslam/odom/body_frc_robot", Odometry, callback=self.tagslam_cb, tcp_nodelay=True, queue_size=1)
        # self.match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data_raw", MatchSpecificData, callback=self.match_data_cb, tcp_nodelay=True, queue_size=1)
        # self.match_data_pub = rospy.Publisher("/frcrobot_rio/match_data", MatchSpecificData, tcp_nodelay=True, queue_size=1)
        # self.override_srv = rospy.Service("enable_manual_override", SetBool, self.manual_override_cb)
        self.raw_fid_pub = rospy.Publisher("/vision/ov2311_10_9_0_9_video0/raw_fiducials", RawFiducialArrayStamped, tcp_nodelay=True, queue_size=1)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.tag_detect_sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_9_video0", ApriltagArrayStamped, self.tag_detect_cb) # TODO: make parameter
        self.cam_info_sub = rospy.Subscriber("/ov2311_10_9_0_9_video0/camera_info", CameraInfo, self.cam_info_cb)
        self.tag_allow_srvs = [rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_9_video0/set_allowed_tags_service", SetAllowedTags), 
                                rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_9_video1/set_allowed_tags_service", SetAllowedTags),
                                rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_10_video0/set_allowed_tags_service", SetAllowedTags),
                                rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_10_video1/set_allowed_tags_service", SetAllowedTags)]
    
    def cam_info_cb(self, msg: CameraInfo):
        self.cam_info = msg
    
    def tag_detect_cb(self, msg: ApriltagArrayStamped):
        if (cam_info != None): 
            # We need to convert ApriltagArrayStamped to RawFiducial (NetworkTables/WPILIB version)
            tags = msg.apriltags
            rfarr = RawFiducialArrayStamped()
            rfarr.raw_fiducials = []
            rfarr.header.stamp = msg.stamp
            rfarr.header.frame_id = msg.frame_id
            for tag in tags:
                raw_fiducial = RawFiducial()
                raw_fiducial.id = tag.id
                # the corners of the apriltag are undistorted, so we take the focal length and principal pt from the 
                # rectified projection matrix of the camera info
                #     [fx'  0  cx' Tx]
                # P = [ 0  fy' cy' Ty]
                #     [ 0   0   1   0]
                raw_fiducial.fx = self.cam_info.P[0]
                raw_fiducial.fy = self.cam_info.P[5]
                # principal point
                raw_fiducial.cx = self.cam_info.P[2]
                raw_fiducial.cy = self.cam_info.P[6]
                raw_fiducial.txnc = Math.atan((tag.center.x - cx)/fx) * 180 / pi # have to convert to degrees
                raw_fiducial.tync = Math.atan((tag.center.y - cy)/fy)
                cam_area = cam_info.height * cam_info.width
                # get area of apriltag from corners
                # using formula for clockwise oriented quadrilateral
                # A=0.5(x1y2+x2y3+x3y4+x4y1)-(x2y1+x3y2+x4y3+x1y4))
                c = tag.corners
                tag_area = 0.5*Math.abs((c[0].x*c[1].y + c[1].x*c[2].y + c[2].x*c[3].y + c[3].x*c[0].y) - 
                                (c[0].y*c[1].x + c[1].y*c[2].x + c[2].y*c[3].x + c[3].y*c[0].x))
                raw_fiducial.ta = tag_area / cam_area
                
                # get transform from camera to tag
                cam_to_tag_tf = self.tf_buf.lookup_transform('cam0', f'tag_{tag.id}', Time(0))
                # get dist (do we need this?)
                x, y, z = cam_to_tag_tf.transform.translation.x, cam_to_tag_tf.transform.translation.y, cam_to_tag_tf.transform.translation.z
                raw_fiducial.distToCamera = Math.sqrt(x**2 + y**2 + z**2)
                
                #TODO: make this accurate...?
                raw_fiducial.ambiguity = 0
                rfarr.raw_fiducials.push(raw_fiducial)
            rospy.loginfo("logging raw fids")
            self.raw_fid_pub.publish(rfarr)
        else:
            rospy.logerr("Not forwarding tag detection data to NetworkTables: Camera info isn't valid!")
    
    # def manual_override_cb(self, req: SetBoolRequest):
    #     if req.data:
    #         rospy.logwarn("tagslam_match_data_republisher: ENABLING MANUAL OVERRIDE. HERE BE DRAGONS. (or unicorns?)")
    #         rospy.logwarn(f"tagslam_match_data_republisher: MANUAL OVERRIDE ENABLED, ALLIANCE COLOR IS CURRENTLY {self.alliance_color_to_string(self.manual_alliance)}")
    #         self.manual_override_enabled = True
    #     else:
    #         rospy.logwarn("tagslam_match_data_republisher: disabling manual override")
    #         self.manual_override_enabled = False
    #     return SetBoolResponse()
    
    # def set_alliance_cb(self, req: OverrideAllianceColorRequest):
    #     self.manual_alliance = req.allianceColor
    #     if self.manual_override_enabled:
    #         rospy.logwarn(f"tagslam_match_data_republisher: MANUAL OVERRIDE ENABLED, ALLIANCE COLOR SET TO {self.alliance_color_to_string(self.manual_alliance)}")
    #     return OverrideAllianceColorResponse()

    # def tagslam_cb(self, msg: Odometry):
    #     if not self.robot_enabled:
    #         if msg.pose.pose.position.x <= FIELD_LENGTH/2:
    #             # rospy.loginfo("valid tagslam blue")
    #             self.tagslam_alliance = MatchSpecificData.ALLIANCE_COLOR_BLUE
    #             self.tagslam_last_x = msg.pose.pose.position.x
    #             self.tagslam_last_time = msg.header.stamp
    #         elif msg.pose.pose.position.x <= FIELD_LENGTH:
    #             # rospy.loginfo("valid tagslam red")
    #             self.tagslam_alliance = MatchSpecificData.ALLIANCE_COLOR_RED
    #             self.tagslam_last_x = msg.pose.pose.position.x
    #             self.tagslam_last_time = msg.header.stamp
    
    # def alliance_color_to_string(self, color: int):
    #     if color == MatchSpecificData.ALLIANCE_COLOR_RED:
    #         return "red"
    #     elif color == MatchSpecificData.ALLIANCE_COLOR_BLUE:
    #         return "blue"
    #     elif color == MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
    #         return "unknown"
    #     else:
    #         return "???????"
    
    # def match_data_cb(self, msg: MatchSpecificData):
    #     self.robot_enabled = msg.Enabled

    #     if not self.robot_enabled and (self.red_ignored or self.blue_ignored):
    #         rospy.loginfo("tagslam_match_data_republisher: enabling all reef tags")
    #         req = SetAllowedTagsRequest()
    #         req.allowed_tags = RED_TAGS + BLUE_TAGS # needed to find alliance
    #         success = True
    #         for tag_allow_srv in self.tag_allow_srvs:
    #             try:
    #                 tag_allow_srv.wait_for_service(timeout=0.01)
    #                 tag_allow_srv.call(req)
    #             except:
    #                 success = False
    #                 rospy.logerr_throttle(1.0, f"tagslam_match_data_republisher: service {tag_allow_srv.resolved_name} is unavailable, trying again next time")
    #         if success:
    #             self.red_ignored = False
    #             self.blue_ignored = False

    #     prev_alliance = msg.allianceColor
    #     if self.manual_override_enabled and self.manual_alliance != MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
    #         if self.manual_alliance != prev_alliance or self.manual_alliance != self.tagslam_alliance:
    #             rospy.logerr_throttle(0.5, f"tagslam_match_data_republisher: ***MANUAL OVERRIDE*** FMS alliance color = {self.alliance_color_to_string(prev_alliance)}, TagSLAM alliance color = {self.alliance_color_to_string(self.tagslam_alliance)}. One of these DOES NOT MATCH MANUAL OVERRIDE = {self.alliance_color_to_string(self.manual_alliance)}. Using MANUAL color {self.alliance_color_to_string(self.manual_alliance)}.")
    #             msg.allianceColor = self.manual_alliance
    #     elif self.tagslam_alliance != MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
    #         if self.tagslam_alliance != prev_alliance:
    #             rospy.logerr_throttle(0.5, f"tagslam_match_data_republisher: FMS alliance color = {self.alliance_color_to_string(prev_alliance)}. This DOES NOT MATCH TagSLAM-based color = {self.alliance_color_to_string(self.tagslam_alliance)}, based on x = {self.tagslam_last_x} at {self.tagslam_last_time}. Using TagSLAM color {self.alliance_color_to_string(self.tagslam_alliance)}.")
    #             msg.allianceColor = self.tagslam_alliance
        
    #     if self.robot_enabled and msg.allianceColor == MatchSpecificData.ALLIANCE_COLOR_RED and (self.red_ignored or not self.blue_ignored):
    #         rospy.loginfo("tagslam_match_data_republisher: enabling red tags, ignoring blue tags")
    #         req = SetAllowedTagsRequest()
    #         req.allowed_tags = RED_TAGS
    #         success = True
    #         for tag_allow_srv in self.tag_allow_srvs:
    #             try:
    #                 tag_allow_srv.wait_for_service(timeout=0.01)
    #                 tag_allow_srv.call(req)
    #             except:
    #                 success = False
    #                 rospy.logerr_throttle(1.0, f"service {tag_allow_srv.resolved_name} is unavailable, trying again next time")
    #         if success:
    #             self.red_ignored = False
    #             self.blue_ignored = True
        
    #     if self.robot_enabled and msg.allianceColor == MatchSpecificData.ALLIANCE_COLOR_BLUE and (self.blue_ignored or not self.red_ignored):
    #         rospy.loginfo("tagslam_match_data_republisher: enabling blue tags, ignoring red tags")
    #         req = SetAllowedTagsRequest()
    #         req.allowed_tags = BLUE_TAGS
    #         success = True
    #         for tag_allow_srv in self.tag_allow_srvs:
    #             try:
    #                 tag_allow_srv.wait_for_service(timeout=0.01)
    #                 tag_allow_srv.call(req)
    #             except:
    #                 success = False
    #                 rospy.logerr_throttle(1.0, f"service {tag_allow_srv.resolved_name} is unavailable, trying again next time")
    #         if success:
    #             self.red_ignored = True
    #             self.blue_ignored = False

    #     self.match_data_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("nt_raw_fid_republisher")
    republisher = RawFiducialRepublisher()  
    rospy.spin()