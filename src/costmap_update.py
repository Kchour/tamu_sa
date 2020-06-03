#!/usr/bin/env python

#now we're importing stuff
import sys
import rospy
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from tamu_sa.msg import ObjectDetect, ObjectDetectArray
import numpy
import tf
import sdl2.ext
import PIL

# numpy.set_printoptions(threshold=sys.maxsize)

global x, y, bb_height, bb_width
# id_number = []
# x = []
# y = []
# bb_height = []
# bb_width = []
counter = 0
node_rate = 5           # in Hz. Determine node speed
resolution = 0.2        # m.
costmap_image =  sdl2.ext.load_image('/home/cast/phoenix-r1/src/mapping/tamu_sa/map/mymap.pgm')
# cp = sdl2.ext.SoftwareSprite(costmap_image,'free') 
costmap_initial = sdl2.ext.pixels2d(costmap_image) 

# costmap_initial = numpy.asarray(PIL.Image.open('/home/cast/phoenix-r1/src/mapping/tamu_sa/map/mymap.pgm'))
# costmap_initial = numpy.array(costmap_initial)

costmap_initial[costmap_initial == 205] = 77
costmap_initial[costmap_initial == 0] = 100
costmap_initial[costmap_initial == 254] = 0

itemindex100 = numpy.where(costmap_initial == 100)
itemindex77 = numpy.where(costmap_initial == 77)

# costmap_initial = numpy.flipud(costmap_initial)

costmap_shape = numpy.shape(costmap_initial)
height = int(numpy.ceil(costmap_shape[0] * resolution)) # in m
width = int(numpy.ceil(costmap_shape[1] * resolution))  # in m
bob = 0



def callback_objectDetect(msg):
    global id_number, x, y, bb_height, bb_width
    # id_number = []
    # x = []
    # y = []
    # bb_height = []
    # bb_width = []
    msg_1 = msg.detections[0]
    x = msg_1.easting
    y = msg_1.northing
    bb_height = msg_1.bb_height
    bb_width = msg_1.bb_width

    # for i in msg.detections:
    #     # msg_i = msg.detections[i]
        # id_number = id_number.append(i.id)
        # x = x.append(i.easting)              # in m
        # y = y.append(i.northing)              # in m
        # bb_height = bb_height.append(i.bb_height)     # in m
        # bb_width = bb_width.append(i.bb_width)       # in m


def main():
    global x, y, bb_height, bb_width
    global counter
    global bob

    rospy.Subscriber("sa_to_costmap", ObjectDetectArray, callback_objectDetect)         # 5 Hz
    costmap_pub = rospy.Publisher("tamu_sa/global_costmap",OccupancyGrid, queue_size=1) # 5 Hz
    rospy.init_node("global_costmap")
    rate = rospy.Rate(node_rate)
    costmap_msg = OccupancyGrid()

    while not rospy.is_shutdown():

        # if x is not None:
        object_array_row_index = int(y/resolution)
        object_array_col_index = int(x/resolution)

        object_left_side = object_array_col_index - int(bb_width/2)
        object_right_side = object_array_col_index + int(bb_width/2)
        object_bottom_side = object_array_row_index - int(bb_height/2)
        object_top_side = object_array_row_index + int(bb_height/2)

        for row in range(object_bottom_side, object_top_side+1):
            for col in range(object_left_side, object_right_side+1):
                costmap_initial[row, col] = 120
                    # print(numpy.shape(costmap_initial))
                    # print('Row:', row)
                    # print('Col:', col)

        costmap = costmap_initial.flatten('C')
        # costmap = 100 - (costmap/(numpy.amax(costmap)) * 100)

        # costmap = costmap
        costmap = costmap.astype(int)

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
        costmap_msg.data = costmap
        costmap_pub.publish(costmap_msg)

        if bob == 0:
            print(numpy.sum(costmap))
            print('77 index:', itemindex77)
            print('100 index:', itemindex100)
            # print(costmap)
            print('Max:', numpy.amax(costmap))
            print('Min:', numpy.amin(costmap))
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
