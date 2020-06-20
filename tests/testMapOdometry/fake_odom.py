#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

rospy.init_node("fake_odom_node")
pubOdom = rospy.Publisher("/husky/odom", Odometry, queue_size=1)

odomMsg = Odometry()
rate = rospy.Rate(15)

# Just publishing the origin for now
while not rospy.is_shutdown():
    odomMsg.header.seq+=1
    odomMsg.header.stamp = rospy.get_rostime()  
    odomMsg.header.frame_id = "husky/odom"  #frame of pose
    odomMsg.child_frame_id = "husky/base"   #frame of twist  
   
    # House locations
    #odomMsg.pose.pose.position.x = 537.62415508
    #odomMsg.pose.pose.position.y = 69.8668
    #odomMsg.pose.pose.position.x = 530.62415508
    #odomMsg.pose.pose.position.y = 103.8668
    
    # Bridge
    #odomMsg.pose.pose.position.x = 0 
    #odomMsg.pose.pose.position.y = 0

    pubOdom.publish(odomMsg)
    rate.sleep()




