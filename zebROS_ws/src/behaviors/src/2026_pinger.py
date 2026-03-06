#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def callback(msg: Float64):
    rospy.loginfo(f"ping returned: {msg.data}")

class Pinger():
    rospy.init_node("pinger")
    pub = rospy.Publisher('ping_send', Float64, queue_size=10)
    rospy.Subscriber('ping_return', Float64, callback)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        ping_time = rospy.get_time()
        rospy.loginfo(f"ping send: {ping_time}")
        pub.publish(Float64(data=ping_time))
        rate.sleep()

    def callback(msg: Float64):
        rospy.loginfo(f"ping returned: {msg.data}")

if __name__ == '__main__':
    pinger = Pinger()  
    rospy.spin()
