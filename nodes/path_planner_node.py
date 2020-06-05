#!/usr/bin/env python
''' Inputs
    /move_base_simple/goal      geometry_msgs/PoseStamped
    /some_map_topic             nav_msgs/OccupancyGrid          Assuming this is a latched topic
    /some_odom_topic            nav_msgs/Odometry
    
    Output
    /path                       nav_msgs/Path
'''

from tamu_sa.search.search_algorithms import AStarSearch
from tamu_sa.search.search_utils import reconstruct_path
from tamu_sa.graphs.graph import SquareGrid
from tamu_sa.graphs.ogm import OccupancyGridMap
from tamu_sa.graphs.grid_utils import get_index
from tamu_sa.graphs.grid_utils import get_world

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry

import time
import pdb

class PathPlannerNode:
    def __init__(self):

        # Initialization
        self.ogrid = None
        self.goalPoint = None
        self.searchSA = None
        self.odom = None
        self.planCounter = 0

        self.subOGM = rospy.Subscriber("/test/map", OccupancyGrid, self.callback_OGM)
        self.subGoal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_Goal)
        self.subOdom = rospy.Subscriber("/husky/odom", Odometry, self.callback_Odom)
        self.pubPath = rospy.Publisher("test/path", Path, queue_size=1, latch=True)

    def get_plan(self):
        '''
        Applies Astar algorithm and returns a linked list 'parent' and cost hash table g
        '''
        # Need to find the nearest cell in grid
        ind = get_index(self.goalPoint[0], self.goalPoint[1], self.grid_size, self.grid_dim)
        goalFilt = get_world(ind[0], ind[1], self.grid_size, self.grid_dim)

        # Need to find the nearest cell in grid
        ind = get_index(self.odom[0], self.odom[1], self.grid_size, self.grid_dim)
        odomFilt = get_world(ind[0], ind[1], self.grid_size, self.grid_dim)

        # Create Search object and use it! Also Time it
        self.searchSA = AStarSearch(self.squareGridGraph, odomFilt, goalFilt, h_type='euclidean', inflation=5.0, visualize=True)
        startTime = time.time()                                         #Start timer
        self.parents, self.g = self.searchSA.use_algorithm()
        self.path = reconstruct_path(self.parents, odomFilt, goalFilt)
        finishTime = time.time() - startTime                            #get finish time
               
        # Create Path msg and pack in data from solution
        self.pathMsg = Path()
        self.pathMsg.header.seq = self.planCounter
        self.pathMsg.header.stamp = rospy.get_rostime() 
        self.pathMsg.header.frame_id= self.mapFrameID
        for ndx, p in enumerate(self.path):
            ps = PoseStamped()
            ps.header.seq = ndx
            ps.header.frame_id = self.mapFrameID
            #ps.header.stamp = rospy.get_rostime() 

            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            self.pathMsg.poses.append(ps)
        
        #publish 
        self.pubPath.publish(self.pathMsg)
        self.planCounter+=1
        
        print("GOT A NEW PLAN in %f secs" % finishTime)

    def callback_Odom(self, msg):
        '''
        expects an Odometry message
        '''
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
     

        #self.counter+=1
        #print("GOT NEW ODOM POINT", self.counter)

    def callback_Goal(self, msg):
        '''
        expects a PoseStamped message
        '''
        # Get prefiltered goal point
        self.goalPoint = (msg.pose.position.x, msg.pose.position.y)

        #print
        print("GOT A NEW GOAL POINT", self.goalPoint)

        if self.odom is not None and self.goalPoint is not None:
            self.get_plan()

    def callback_OGM(self, msg):
        ''' 
        expects an OccupancyGrid Message 
        
        (Trinary values)
        If p > occupied_thresh, output the value 100 to indicate the cell is occupied.
        If p < free_thresh, output the value 0 to indicate the cell is free.
        Otherwise, output -1 a.k.a. 255 (as an unsigned char), to indicate that the cell is unknown. 

        '''
        # Consider ogrid to binary ogm, but using trinary values should still work
        self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.mapFrameID = msg.header.frame_id

        # Boundaries of map in physical coordinates
        self.minX = self.origin[0]
        self.maxX = msg.info.resolution * msg.info.width + self.origin[0] - 1
        self.minY = self.origin[1]
        self.maxY = msg.info.resolution * msg.info.height + self.origin[1] - 1
        
        # store some pertinent values for reuse
        self.grid_dim = (self.minX, self.maxX, self.minY, self.maxY)
        self.grid_size = msg.info.resolution

        # ================== Consider Rebinning here ========================#
        
        #factor = 2
        # newShape = (msg.info.height//factor, msg.info.width//factor)
        # rebin(self.ogrid, newShape)        
        #self.ogrid = bin_ndarray(self.ogrid, (msg.info.height//factor, msg.info.width//factor))
        
        # use SquareGrid class
        self.squareGridGraph = SquareGrid(self.ogrid, grid_dim=self.grid_dim, grid_size=self.grid_size, type_=8)

        print("GOT A NEW MAP")
        # (class graph, tuple start, tuple goal, string h_type, bool visualize)
        if self.odom is not None and self.goalPoint is not None: 
            self.get_plan()

if __name__=="__main__":
        rospy.init_node("planner_test_node")
        ppn = PathPlannerNode()

        # Debug
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.gca()
        im = ax.imshow(ppn.ogrid, origin='lower', extent=[ppn.minX, ppn.maxX, ppn.minY, ppn.maxY])
        plt.axis('equal')
        #fig.canvas.draw() 
        #plt.show(block=False)
        plt.show()

        rospy.spin()
