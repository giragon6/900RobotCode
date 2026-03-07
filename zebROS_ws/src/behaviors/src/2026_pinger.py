#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

def callback(msg: Float64):
    rospy.loginfo(f"ping returned: {msg.data}")

class Pinger():
    def __init__(self):
        rospy.init_node("pinger")
        pub = rospy.Publisher('ping_send', Float64, queue_size=10)
        rospy.Subscriber('ping_return', Float64, self.callback)

        rate = rospy.Rate(0.05)  # 0.05 Hz = once every 20s
        # rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            ping_time = rospy.get_time()
            rospy.loginfo(f"ping send: {ping_time}")
            pub.publish(Float64(data=ping_time))
            rate.sleep()

    def callback(self, msg: Float64):
        rospy.loginfo(f"ping returned: {msg.data}")
        

if __name__ == '__main__':
    pinger = Pinger()  
    rospy.spin()
