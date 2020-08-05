#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class FakeOdom(object):
	def __init__(self, x_desired=0, y_desired=0, header_id="test/odom", child_id = "test/base"):
		self.x = x_desired
		self.y = y_desired

	def run_odom(self):

		rospy.init_node("fake_odom_node")
		pubOdom = rospy.Publisher("/test/odom", Odometry, queue_size=1)

		odomMsg = Odometry()
		rate = rospy.Rate(15)

		# Just publishing the origin for now
		while not rospy.is_shutdown():
			odomMsg.header.seq+=1
			odomMsg.header.stamp = rospy.get_rostime()  
			odomMsg.header.frame_id = "test/odom"  #frame of pose
			odomMsg.child_frame_id = "test/base"   #frame of twist  
			
			#odomMsg.pose.pose.position.x = 2
			#odomMsg.pose.pose.position.y = -94.39635
			odomMsg.pose.pose.position.x = self.x
			odomMsg.pose.pose.position.y = self.y

			pubOdom.publish(odomMsg)
			rate.sleep()


if __name__=="__main__":
	x = 2
	y = -94.39635
	fo = FakeOdom(x,y)
	fo.run_odom()



