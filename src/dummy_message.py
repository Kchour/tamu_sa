#!/usr/bin/env python

#now we're importing stuff
import sys
import rospy
import numpy
import tf

from tamu_sa.msg import ObjectDetectArray, ObjectDetect


def main():

    costmap_pub = rospy.Publisher("sa_to_costmap", ObjectDetectArray, queue_size=10)

    rospy.init_node("dummy_object_message")
    
    rate = rospy.Rate(10)

    costmap_msg = ObjectDetectArray()

    cur_det = ObjectDetect()
    cur_det.easting = 100
    cur_det.northing = 200
    cur_det.id = 1
    cur_det.bb_height = 5
    cur_det.bb_width = 5
    cur_det.obj_confidence = 1
    costmap_msg.detections.append(cur_det)

    while not rospy.is_shutdown():
        costmap_pub.publish(costmap_msg)
        rate.sleep()





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
    finally:
        print("we shutdown! :)")