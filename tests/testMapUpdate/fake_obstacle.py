#!/usr/bin/env python

import rospy
import numpy as np
from tamu_sa.msg import ObjectDetectArray, ObjectDetect

'''
This test node publishes the position of 2 moving obstacle on messag
e type
<ObjectDetectArray>
'''

#objectArrayMsg = ObjectDetectArray()
objectArrayDict = {}

def return_object_array():
    ''' Construct message '''
    msg = ObjectDetectArray()
    msg.detections = list(objectArrayDict.values())
    return msg

def delete_object(id):
    del objectArrayDict[id]

def add_object(id, f_obstacles):
    objectArrayDict[id] = f_obstacles.newObject

class FakeObstacles:
    def __init__(self, easting, northing, id, bb_height, bb_width, obj_confidence):
        # cur_det = ObjectDetect()
        # cur_det.easting = 100
        # cur_det.northing = 300
        # cur_det.id = 134
        # cur_det.bb_height = 25
        # cur_det.bb_width = 25
        # cur_det.obj_confidence = 1

        self.newObject = ObjectDetect()
        self.newObject.easting = easting
        self.newObject.northing = northing
        self.newObject.id = id
        self.newObject.bb_height = bb_height
        self.newObject.bb_width = bb_width
        self.newObject.obj_confidence = obj_confidence

        self.init_easting = easting
        self.init_northing = northing

        #objectArrayMsg.detections.append(self.newObject)
        objectArrayDict[id] = self.newObject
    
    def get_object(self):
        return self.newObject

    def update_object(self, easting, northing, bb_height, bb_width):
        self.newObject.easting = easting
        self.newObject.northing = northing
        self.newObject.bb_height = bb_height
        self.newObject.bb_width = bb_width

class FakeObstacleNode:
    def __init__(self):
        
        self.obstacle_reserve = {}  # 

        self.objPub = rospy.Publisher("/test/object_detection", ObjectDetectArray, queue_size=1)

        rate = rospy.Rate(10)

        # Each time we create an obstacle, it gets pushed back to a global array
        #obstacle1 = FakeObstacles(100, -80, 134, 25, 25, 1)     # Moving left/right
        obstacle2 = FakeObstacles(42, -25, 2, 10, 10, 1)        # Moving up/down
        #obstacle3 = FakeObstacles(110, -80, 10, 20, 20, 1)      # Stationary / POP-UP

        initTime = rospy.get_time()
        while not rospy.is_shutdown():

            timeNow = rospy.get_time() - initTime
            #self.use_motion_model_x(obstacle1, timeNow, 10, 25)
            #self.use_motion_model_y(obstacle2, timeNow, 10, 25)

            self.pop_up(obstacle2, timeNow, 45)
            #self.pop_up(obstacle3, timeNow, 10)

            self.objPub.publish(return_object_array())
            rate.sleep()

    def use_motion_model_x(self,obstacle, t, per, amp):
        # move left/right 25 meters
        new_x = amp*np.sin(2*np.pi/per*t)
        obstacle.newObject.easting = new_x + obstacle.init_easting

    def use_motion_model_y(self, obstacle, t, per, amp):
        # move up and down 25 meters
        new_y = amp*np.sin(2*np.pi/per*t)
        obstacle.newObject.northing = new_y + obstacle.init_northing
    
    def pop_up(self, f_obstacle, t, tAppear):
        ''' obstacle will pop up after tAppear seconds '''
        if t > tAppear:
            if f_obstacle.newObject.id in self.obstacle_reserve:
                add_object(f_obstacle.newObject.id, f_obstacle)
                del self.obstacle_reserve[f_obstacle.newObject.id]
        else:
            if f_obstacle.newObject.id in objectArrayDict:
                self.obstacle_reserve[f_obstacle.newObject.id] = f_obstacle.newObject
                delete_object(f_obstacle.newObject.id)

if __name__=="__main__":
        rospy.init_node("fake_obstacle_detect")
        fod = FakeObstacleNode()
        rospy.spin()
