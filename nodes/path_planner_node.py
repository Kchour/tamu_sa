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
from tamu_sa.graphs.grid_utils import pooling
from tamu_sa.costmap.costmap_layer import CostMap

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from tamu_sa.msg import ObjectDetectArray, ObjectDetect

import time
import pdb
from threading import Thread, Lock
mutex = Lock()

# Debug
import matplotlib.pyplot as plt

class PathPlannerNode:
    def __init__(self):

        # ===================== Initialization ========================

        # init grid arrays
        self.sgrid = None      # static grid
        self.ogrid = None      # obstacle grid
        self.costMap = CostMap()

        # other init variables
        self.goalPoint = None
        self.searchSA = None
        self.odom = None
        self.planCounter = 0

        # ======= States =========
        # StaticLayer published? yes/no
        # ObstacleLayer published? yes/no
        # Global Path gen? yes/no
        # Replanned once? yes/no
        self.pathGlobalGen = False
        self.pathReplanOnce = False

        self.subOGM = rospy.Subscriber("/test/map", OccupancyGrid, self.callback_OGM)
        self.subGoal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_Goal)
        self.subOdom = rospy.Subscriber("/husky/odom", Odometry, self.callback_Odom)
        self.pubPath = rospy.Publisher("/test/path", Path, queue_size=1, latch=True)

        #TODO Consider publishing total cost map for debug visualization purposes
        self.pubCostMap = rospy.Publisher("/test/full_map", OccupancyGrid, queue_size=1, latch=True)

        # Obstacle detection with <ObjectDetectArray>
        self.subObjectDetect = rospy.Subscriber("/test/object_detection",  ObjectDetectArray, self.callback_OD)

    # CALL IN MAIN 
    def publish_full_map(self):
        '''Publishes an OccupancyGrid type message'''

        print("Publishing Full Map")
        rate = rospy.Rate(5)
        # empty grid / pose messages for speed?
        self.ogmMsg = OccupancyGrid()
        pose = Pose()

        while not rospy.is_shutdown():
            if self.costMap.does_layer_exist(['StaticLayer', 'ObstacleLayer']):
                fullMap = self.costMap.return_total_map()
                m, n = fullMap.shape
                self.ogmMsg.header.seq += 1
                self.ogmMsg.header.stamp = rospy.get_rostime()
                self.ogmMsg.header.frame_id = self.mapFrameID

                # Fill in meta data
                self.ogmMsg.info.map_load_time = self.ogmMsg.header.stamp
                self.ogmMsg.info.resolution = self.grid_size
                self.ogmMsg.info.width = n   #cells
                self.ogmMsg.info.height = m  #cells
                # fill in pose data for origin
                pose.position.x = self.origin[0]
                pose.position.y = self.origin[1]
                pose.orientation.w = 1
                self.ogmMsg.info.origin = pose
                # Flatten numpy array row-majored
                #pdb.set_trace()
                self.ogmMsg.data = fullMap.flatten('C').astype(np.int8).tolist()
                self.pubCostMap.publish(self.ogmMsg)
            rate.sleep()

    def publish_path(self, path):
        ''' Publish a nav_msgs/Path message latched topic '''
        # TODO: Insanely slow, need to replace loop someday
        self.pathMsg = Path()
        self.pathMsg.header.seq  +=1
        self.pathMsg.header.stamp = rospy.get_rostime() 
        self.pathMsg.header.frame_id = self.mapFrameID
        for ndx, p in enumerate(path):
            ps = PoseStamped()
            ps.header.seq = ndx
            ps.header.frame_id = self.mapFrameID
            #ps.header.stamp = rospy.get_rostime() 

            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            self.pathMsg.poses.append(ps)
        
        self.pubPath.publish(self.pathMsg)

    
    def re_plan(self):
        ''' Replan only segments which have an obstacle, and after generating a new path '''
        if self.pathGlobalGen:

            # Detect path in obstacles (collision)
            fullmap = self.costMap.return_total_map()
            indx, indy = get_index(self.path[:,0], self.path[:,1], self.grid_size, self.grid_dim)
            #fullmap[ind[1], ind[0]]
            # Maskbit true if an obstacle (100) and  TODO beyond some distance
            #mask = (fullmap[ind[1], ind[0]]==100)
            mask = np.diff(fullmap[indy, indx]>=100, prepend=False)
            maskTrueInd = np.where(mask==True)
            offset = 20      # in terms of index
            try:
                if len(maskTrueInd[0]) > 0:
                    # TODO: Find a better way to modify the indCor here to be further away
                    offsetMask = np.zeros_like(mask)
                    offsetMaskInd = [np.clip(maskTrueInd[0][0] - offset, 0, len(mask)-1 ), np.clip(maskTrueInd[0][1]+offset, 0, len(mask)-1)]
		    
                    offsetMask[offsetMaskInd] = True
                    # Stack indices
                    #indCor = np.vstack((indx[mask],indy[mask])).T
                    indCor = np.vstack((indx[offsetMask], indy[offsetMask])).T

                    # Convert to worldcor
                    worldCorX, worldCorY = get_world(indCor[:,0], indCor[:,1], self.grid_size, self.grid_dim)

                    # look for places that change using np.diff
                    #maskdiff = (np.diff(mask)==True)
                    #fullmap[ind[1][maskdiff], ind[0][maskdiff]]
                    
                    # Pad the truth?
                    #maskTrue = np.where(mask==True)

                    # introduce for loop for multiple obstacles (each obstacle has an entering and exiting segment) (each obstacle has 2 trues)
                    #startPoint = (worldCorX[0], worldCorY[0])
                    startInd = get_index(self.odom[0], self.odom[1], self.grid_size, self.grid_dim)
                    startPoint = get_world(startInd[0], startInd[1], self.grid_size, self.grid_dim)
                    goalPoint = (worldCorX[1], worldCorY[1])

                    # Then replan if needed
                    self.searchSA_replan = AStarSearch(self.squareGridGraphTotal, startPoint, goalPoint, h_type='euclidean', inflation=5.0, visualize=False)
                    #startTime = time.time()                                         #Start timer
                    parents, g = self.searchSA_replan.use_algorithm()
                    newSegment = reconstruct_path(parents, startPoint, goalPoint)
                    #finishTime = time.time() - startTime
                    
                    # insert new segment into global path
                    # self.path[maskTrueInd[0][0]:maskTrueInd[0][1], :] = np.nan
                    # self.path = np.insert(self.path, maskTrueInd[0][0]+1, newSegment, 0)     
                    #self.path[offsetMaskInd[0]:offsetMaskInd[1], :] = np.nan
                    #self.path = np.insert(self.path, offsetMaskInd[0]+1, newSegment, 0)    
                    #self.path =  self.path[~np.isnan(self.path).any(axis=1)]            #remove nans
                    self.path = np.vstack((newSegment, self.path[offsetMaskInd[1]:-1]))
                    self.publish_path(self.path)

#		    self.publish_path(newSegment)
                    self.planCounter += 1
                    #print(self.planCounter, "REPLANNING")
		    #print(maskTrueInd)
            except Exception as err:
                print(err)
                pdb.set_trace()
                

    def get_plan(self):
        '''
        Applies Astar algorithm and returns a linked list 'parent' and cost hash table g.
        Called when new goal is given
        '''
        mutex.acquire() 
        # Need to find the nearest cell in grid
        ind = get_index(self.goalPoint[0], self.goalPoint[1], self.grid_size, self.grid_dim)
        goalFilt = get_world(ind[0], ind[1], self.grid_size, self.grid_dim)

        # Need to find the nearest cell in grid
        ind = get_index(self.odom[0], self.odom[1], self.grid_size, self.grid_dim)
        odomFilt = get_world(ind[0], ind[1], self.grid_size, self.grid_dim)

        # Create Search object and use it! Also Time it. TODO Reverse Search Direction
        self.searchSA = AStarSearch(self.squareGridGraphStatic, odomFilt, goalFilt, h_type='euclidean', inflation=1.0, visualize=False)

         # debug
        if self.searchSA.visualize:
            self.fig = plt.figure(1)
            self.ax = self.fig.gca()
            plt.axis('equal')
            self.ax.clear()
            self.im = self.ax.imshow(self.costMap.return_total_map(), origin='lower', extent=[self.minX, self.maxX, self.minY, self.maxY])

        startTime = time.time()                                         #Start timer
        self.parents, self.g = self.searchSA.use_algorithm()
        self.path = reconstruct_path(self.parents, odomFilt, goalFilt)
        finishTime = time.time() - startTime                            #get finish time

        # Publish
        self.publish_path(self.path)       
        print("GOT A NEW PLAN in %f secs" % finishTime)

        self.pathGlobalGen = True
        mutex.release()

    # ========== Callback functions below ============    

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
        self.sgrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.mapFrameID = msg.header.frame_id

        # Boundaries of map in physical coordinates
        self.minX = self.origin[0]
        self.maxX = msg.info.resolution * msg.info.width + self.origin[0] - 1
        self.minY = self.origin[1]
        self.maxY = msg.info.resolution * msg.info.height + self.origin[1] - 1
        
        # Set all unknowns (-1) to occupied (100)
        #self.sgrid[(self.sgrid==-1)] = 100

        # ================== Consider Rebinning here ========================#
        
        #factor = 2
        # newShape = (msg.info.height//factor, msg.info.width//factor)
        # rebin(self.ogrid, newShape)        
        #self.ogrid = bin_ndarray(self.ogrid, (msg.info.height//factor, msg.info.width//factor))
        
        newRes = 1
        ksize = np.round(newRes/msg.info.resolution).astype(int)
        self.sgrid = pooling(self.sgrid, (ksize, ksize), pad=True)

        # store some pertinent values for reuse
        self.grid_dim = (self.minX, self.maxX, self.minY, self.maxY)
        #self.grid_size = msg.info.resolution
        self.grid_size = newRes     

        # update layer (note that we can directly manipulate layer variable )
        # or manipulate self.sgrid directly!
        if not self.costMap.does_layer_exist("StaticLayer"): 
            self.costMap.add_layer(self.sgrid, name="StaticLayer")

        # use SquareGrid class, "pointers" in use! Generate a good global plan on the sgrid
        self.squareGridGraphStatic = SquareGrid(self.sgrid, grid_dim=self.grid_dim, grid_size=self.grid_size, type_=8)
        #self.squareGridGraph = SquareGrid(self.costMap.return_total_map(), grid_dim=self.grid_dim, grid_size=self.grid_size, type_=8)

        print("GOT A NEW MAP")
        # (class graph, tuple start, tuple goal, string h_type, bool visualize)
        if self.odom is not None and self.goalPoint is not None: 
            self.get_plan()

    def callback_OD(self, msg):
        '''
        Expects an ObjectDetectArray, which is a list of ObjectDetect's
        # cur_det = ObjectDetect()
        # cur_det.easting = 100
        # cur_det.northing = 300
        # cur_det.id = 134
        # cur_det.bb_height = 25
        # cur_det.bb_width = 25
        # cur_det.obj_confidence = 1

        currently assumes like frame of map
        '''

        # TODO: DO SOME FRAME TRANSFORMATIONS HERE TOO 
        # TODO: IF no updates, clear the obstacles from self.ogrid
        # TODO: Error handling if detection is outside of the map!

        # Only Add a layer once, then update the existing layer subsequently
        # obstacle layer has to be the same grid size, grid dim as static layer
        if not self.costMap.does_layer_exist("ObstacleLayer") and self.costMap.does_layer_exist("StaticLayer"):

            self.ind_func = lambda d: get_index(d[0],d[1], self.grid_size, self.grid_dim)
            self.ogrid = np.zeros_like(self.sgrid)
            self.costMap.add_layer(self.ogrid, "ObstacleLayer")
            print("Added Obstacle Layer")
        elif self.costMap.does_layer_exist(["StaticLayer","ObstacleLayer" ]):
            self.ogrid[:] = np.zeros_like(self.sgrid)
            for od in msg.detections:
                # od is an ObjectDetect message type

                cx = od.easting
                cy = od.northing

                # Corners of bounding box
                topLeft = (cx - od.bb_width/2, cy + od.bb_height/2)
                topRight = (cx + od.bb_width/2, cy + od.bb_height/2)
                bottomLeft = (cx - od.bb_width/2, cy - od.bb_height/2)
                bottomRight = (cx + od.bb_width/2, cy - od.bb_height/2)
                
                # Everything in the bounding box is an obstacle! 
                mapInd = list(map(self.ind_func, (topLeft, topRight, bottomLeft, bottomRight)))
                minInd, maxInd = min(mapInd), max(mapInd)
                self.ogrid[minInd[1]:maxInd[1], minInd[0]:maxInd[0]] = 100.0

                self.squareGridGraphTotal = SquareGrid(self.costMap.return_total_map(), grid_dim=self.grid_dim, grid_size=self.grid_size, type_=8)
                self.re_plan()

if __name__=="__main__":
        rospy.init_node("planner_test_node")
        ppn = PathPlannerNode()

        plt.show(block=False)   #Debug plotter may not work properly
        ppn.publish_full_map()

        rospy.spin()        #may be unncessary
