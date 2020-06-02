#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Switcharoo:
    def __init__(self):
        rospy.init_node("republisher_node")
        self.newMsg = PoseStamped()

        self.pub = rospy.Publisher("/sa_gv_info", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber("/husky/odom", Odometry, self.callback)
        
        rospy.spin() 
    
    def callback(self, msg):
        self.newMsg.header = msg.header
        self.newMsg.pose = msg.pose.pose
        self.pub.publish(self.newMsg)

if __name__ == '__main__':
    try:
        sw = Switcharoo()
    except rospy.ROSInterruptException: 
        pass
