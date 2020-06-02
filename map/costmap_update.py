#!/usr/bin/env python

#now we're importing stuff
import sys
import rospy
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
import numpy
import tf
import sdl2.ext
import PIL

# numpy.set_printoptions(threshold=sys.maxsize)

counter = 0
node_rate = 5           # in Hz. Determine node speed
resolution = 0.2        # m.
# costmap_image =  sdl2.ext.load_image("mymap.pgm")
# cp = sdl2.ext.SoftwareSprite(costmap_image,'free') 
# costmap_initial = sdl2.ext.pixels2d(costmap_image) 

costmap_initial = numpy.asarray(PIL.Image.open(b'mymap.pgm'))
costmap_initial = numpy.array(costmap_initial)

#numpy.where(costmap_initial == 254, costmap_initial, 100)
costmap_initial[costmap_initial == 205] = 77
costmap_initial[costmap_initial == 0] = 100
costmap_initial[costmap_initial == 254] = 0

itemindex100 = numpy.where(costmap_initial == 100)
itemindex77 = numpy.where(costmap_initial == 77)



costmap_shape = numpy.shape(costmap_initial)
height = int(numpy.ceil(costmap_shape[0] * resolution)) # in m
width = int(numpy.ceil(costmap_shape[1] * resolution))  # in m
bob = 0



# def show_img(data):
#     try:
#         dumb_img = CvBridge().imgmsg_to_cv2(data.seg_img, "passthrough")
#     except CvBridgeError as e:
#         print(e)
#     cv2.imshow("Subscribed image", dumb_img)
#     cv2.waitKey(1)


def main():
    sub = rospy
    pub = rospy.Publisher("/husky/global_planner/costmap/costmap",OccupancyGrid, queue_size=1)
    rospy.init_node("global_costmap")
    rate = rospy.Rate(node_rate)
    costmap_msg = OccupancyGrid()
    global counter
    global bob


    while not rospy.is_shutdown():
        costmap = costmap_initial.flatten('C')

        # costmap = 100 - (costmap/(numpy.amax(costmap)) * 100)

        costmap = costmap
        costmap = costmap.astype(int)


        # costmap = costmap-128

        costmap_msg.header.seq = counter
        costmap_msg.header.stamp  = rospy.get_rostime()
        #costmap_msg.header.frame_id = 0
        costmap_msg.info.width = costmap_shape[1]
        costmap_msg.info.height = costmap_shape[0]
        costmap_msg.info.resolution = resolution

        #setting the origin this way makes it easy to visualize in rviz along
        #with the pointcloud, but we'll probably want to redo this once it's 
        #actually on the car.  I suspect we'll use tf somehow, but I don't
        #really know how to use tf.
        # costmap_msg.info.origin.position.x = half_map_length
        # costmap_msg.info.origin.position.y = -half_map_length
        costmap_msg.info.origin.position.x = 0
        costmap_msg.info.origin.position.y = 0
        costmap_msg.info.origin.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0,0,numpy.pi/2)
        costmap_msg.info.origin.orientation.x = quaternion[0]
        costmap_msg.info.origin.orientation.y = quaternion[1]
        costmap_msg.info.origin.orientation.z = quaternion[2]
        costmap_msg.info.origin.orientation.w = quaternion[3]
        #costmap_msg.info.map_load_time = idk
        costmap_msg.data = costmap
        pub.publish(costmap_msg)

        if bob == 0:
            # print(costmap_initial)
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


            bob = bob + 1    
        # print(numpy.sum(costmap))

        counter = counter+1
        rate.sleep

        # print(numpy.shape(cost_map_initial))





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
    finally:
        print("we shutdown! :)")

