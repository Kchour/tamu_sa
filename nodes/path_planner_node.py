#!/usr/bin/env python

import rospy


from tamu_sa.search.search_algorithms import AStarSearch
from tamu_sa.graphs.graph import SquareGrid
from tamu_sa.graphs.ogm import OccupancyGridMap


rospy.init_node("planner_test_node")

rospy.spin()