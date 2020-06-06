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

    # cur_det = ObjectDetect()
    # cur_det.easting = 100
    # cur_det.northing = 300
    # cur_det.id = 134
    # cur_det.bb_height = 25
    # cur_det.bb_width = 25
    # cur_det.obj_confidence = 1
    # costmap_msg.detections.append(cur_det)

    # cur_det2 = ObjectDetect()
    # cur_det2.easting = 100
    # cur_det2.northing = 100
    # cur_det2.id = 2
    # cur_det2.bb_height = 10
    # cur_det2.bb_width = 30
    # cur_det2.obj_confidence = 1
    # costmap_msg.detections.append(cur_det2)



    while not rospy.is_shutdown():
        costmap_pub.publish(costmap_msg)
        rate.sleep()





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
    finally:
        print("we shutdown! :)")