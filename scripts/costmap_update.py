#!/usr/bin/env python

#now we're importing stuff
# import sys
# import PIL
import rospy
from nav_msgs.msg import OccupancyGrid
from tamu_mtt.msg import ObjectDetect, ObjectDetectArray
import numpy
import tf
import sdl2.ext
import yaml
import os

global costmap_initial
global x, y, bb_height, bb_width, id_number
id_number = numpy.asarray([])
x = numpy.asarray([])
y = numpy.asarray([])
bb_height = numpy.asarray([])
bb_width = numpy.asarray([])

# Read yaml file
path = os.path.dirname(os.path.abspath(__file__))
map_path = path +'/../map'

counter = 0
node_rate = 5           # in Hz. Determine node speed
resolution = 0.2        # m.
# Convert the image to an array
costmap_image =  sdl2.ext.load_image('/home/cast/phoenix-r1/src/mapping/tamu_sa/map/mymap.pgm')
costmap_initial = sdl2.ext.pixels2d(costmap_image) 
costmap_initial = numpy.array(costmap_initial)

costmap_initial[costmap_initial == 205] = 77    # unknown space
costmap_initial[costmap_initial == 0] = 100     # free space
costmap_initial[costmap_initial == 254] = 0     # obstacle

# itemindex100 = numpy.where(costmap_initial == 100)
# itemindex77 = numpy.where(costmap_initial == 77)

# Collect the row and columns in the array
costmap_shape = numpy.shape(costmap_initial)

bob = 0

def callback_objectDetect(msg):
    global id_number, x, y, bb_height, bb_width
    id_number = numpy.asarray([])
    x = numpy.asarray([])
    y = numpy.asarray([])
    bb_height = numpy.asarray([])
    bb_width = numpy.asarray([])

    for i in msg.detections:
        id_number = numpy.append(id_number, i.id)
        x = numpy.append(x , i.easting)              # in m
        y = numpy.append(y, i.northing)              # in m
        bb_height = numpy.append(bb_height, i.bb_height)     # in m
        bb_width = numpy.append(bb_width, i.bb_width)       # in m



def main():
    global x, y, bb_height, bb_width, id_number
    global counter
    global bob, costmap_initial

    rospy.Subscriber("/sa_to_costmap", ObjectDetectArray, callback_objectDetect)         # 5 Hz
    costmap_pub = rospy.Publisher("/tamu_sa/global_costmap",OccupancyGrid, queue_size=1) # 5 Hz
    rospy.init_node("global_costmap")
    rate = rospy.Rate(node_rate)
    costmap_msg = OccupancyGrid()

    while not rospy.is_shutdown():
        # costmap_edit = costmap_initial
        costmap_edit = numpy.empty_like(costmap_initial)
        costmap_edit[:] = costmap_initial 

        if len(id_number) < 1:
            print('NO OBJECT')
            print("id:",id_number)
            pass
        else:
            print('x', x)
            object_array_row_index = (numpy.true_divide(y, resolution)).astype(int)
            object_array_col_index = (numpy.true_divide(x, resolution)).astype(int)

            object_left_side = object_array_col_index - (numpy.true_divide(bb_width, 2.0*resolution)).astype(int)
            object_right_side = object_array_col_index + (numpy.true_divide(bb_width, 2.0*resolution)).astype(int)
            object_bottom_side = object_array_row_index - (numpy.true_divide(bb_height, 2.0*resolution)).astype(int)
            object_top_side = object_array_row_index + (numpy.true_divide(bb_height, 2.0*resolution)).astype(int)

            for i in range(len(object_left_side)):
                for row in range(object_bottom_side[i], object_top_side[i]+1):
                    for col in range(object_left_side[i], object_right_side[i]+1):
                        costmap_edit[row, col] = 120


        costmap_final = costmap_edit.flatten('C')

        # costmap = costmap
        costmap_final = costmap_final.astype(int)

        costmap_msg.header.seq = counter
        costmap_msg.header.stamp  = rospy.get_rostime()
        #costmap_msg.header.frame_id = 0
        costmap_msg.info.width = costmap_shape[1]
        costmap_msg.info.height = costmap_shape[0]
        costmap_msg.info.resolution = resolution

        # costmap_msg.info.origin.position.x = 48.500002
        # costmap_msg.info.origin.position.y = 148.900003
        costmap_msg.info.origin.position.x = -100
        costmap_msg.info.origin.position.y = -200
        costmap_msg.info.origin.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        costmap_msg.info.origin.orientation.x = quaternion[0]
        costmap_msg.info.origin.orientation.y = quaternion[1]
        costmap_msg.info.origin.orientation.z = quaternion[2]
        costmap_msg.info.origin.orientation.w = quaternion[3]
        #costmap_msg.info.map_load_time = idk
        costmap_msg.data = costmap_final
        costmap_pub.publish(costmap_msg)

        if bob == 0:
            print(numpy.sum(costmap_final))
            # print('77 index:', itemindex77)
            # print('100 index:', itemindex100)
            # print(costmap)
            print('Max:', numpy.amax(costmap_final))
            print('Min:', numpy.amin(costmap_final))
            print('Integer height:', height)
            print('Integer Width:', width)
            print('Height:', costmap_shape[0] * resolution)
            print('Width:', costmap_shape[1] * resolution)
            print ('Easting:', x)

            bob = bob + 1    

        counter = counter+1
        rate.sleep

        # print(numpy.shape(cost_map_initial))





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
    finally:
        print("we shutdown! :)")
