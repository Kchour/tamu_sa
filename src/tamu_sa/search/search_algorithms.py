#!/usr/bin/env python3
''' Independent '''
from .search_utils import PriorityQueue
from tamu_sa.graphs.grid_utils import get_index
from tamu_sa.graphs.grid_utils import get_world

from tamu_sa import run_config as rc
if rc.visualize:
    import matplotlib.pyplot as plt
import numpy as np
import time

''' Dependecies '''
from tamu_sa.animation.animation import Animate

import pdb

class Search:
    def __init__(self, graph, start, goal):
        self.graph = graph
        self.start = start
        self.goal = goal

    def set_start(self, start):
        self.start = start

    def set_goal(self, goal):
        self.goal = goal

class AStarSearch(Search):
    def __init__(self, graph, start, goal, h_type, inflation = 1.0, visualize=False):
        Search.__init__(self, graph, start, goal)
        self.h_type = h_type
        self.inflation = inflation
        self.visualize = visualize

        # A star initialize openList, closedList
        self.frontier = PriorityQueue()       # The OPENLIST
        self.frontier.put(self.start, 0)      # PUT START IN THE OPENLIST
        self.parent = {}              # parent, {loc: parent}
        # g function dict, {loc: f(loc)}, CLOSED LIST BASICALLY
        self.g = {}
        self.parent[self.start] = None
        self.g[self.start] = 0
        
        # Visulization?
        if self.visualize:     
            # initialize plot (graph has the dimensions built it)
            xlim = (graph.grid_dim[0], graph.grid_dim[1])   #(minX, maxX)
            ylim = (graph.grid_dim[2], graph.grid_dim[3])   #(minY, maxY)
            self.grid_size = graph.grid_size
            self.grid_dim = (xlim[0], xlim[1], ylim[0], ylim[1])
            # 50 millisecond sleep
            self.animateCurrent = Animate(number=1, xlim=xlim, ylim=ylim, gridSize=1,linewidth=5, markerType='xc', markerSize=10, sleep=0.000, order=10)
            self.animateNeighbors = Animate(number=1, xlim=xlim, ylim=ylim, gridSize=1,linewidth=5, markerType='o', markerSize=5, sleep=0.000, order=-1)
            self.animatePath = Animate(number=1, xlim=xlim, ylim=ylim, gridSize=1,linewidth=5, markerType='o', markerSize=5, sleep=0.000, order=-1)


    ''' heuristic function '''
    def heuristic(self, a, b, type_='manhattan'):
        '''
        General Idea
        4 neighbors:    manhattan
        8 neighbors:    diagonal
        any direction:  euclidean
        '''
        (x1, y1) = a
        (x2, y2) = b
        if type_ == 'manhattan':
            return abs(x1 - x2) + abs(y1 - y2)
        elif type_ == 'euclidean':
            v = [x2 - x1, y2 - y1]
            return np.hypot(v[0], v[1])
        elif type_ == 'diagonal_uniform':
            return max(abs(x1 - x2), abs(y1 - y2))
        elif type_ == 'diagonal_nonuniform':
            dmax = max(abs(x1 - x2), abs(y1 - y2))
            dmin = min(abs(x1 - x2), abs(y1 - y2))
            return 1.414*dmin + (dmax - dmin)
    
    def use_algorithm(self):
        ''' Usage:
            - call to runs full algorithm until termination

            Returns:
            - a linked list, 'parent'
            - hash table of nodes and their associated min cost, 'g'
        '''
        
        frontier = self.frontier
        parent = self.parent
        g = self.g

        while not frontier.empty():
            current = frontier.get()  # update current to be the item with best priority

            if self.visualize:
                # Update plot with visuals
                self.animateCurrent.update(current)
                #self.animateCurrent.update(get_world(current[0], current[1], self.grid_size, self.grid_dim))

            # early exit if we reached our goal
            if current == self.goal:
                break
            # expand current node and check neighbors
            for next in self.graph.neighbors(current):
                g_next = g[current] + self.graph.cost(current, next)
                # if next location not in CLOSED LIST or its cost is less than before
                # Newer implementation
                if next not in g or g_next < g[next]:
                    g[next] = g_next
                    if self.h_type == 'zero' or self.goal == None:
                        priority = g_next 
                    else:
                        priority = g_next + self.inflation*self.heuristic(self.goal, next, self.h_type)
                    frontier.put(next, priority)
                    parent[next] = current

                    if self.visualize:
                        self.animateNeighbors.update(next)
                        #self.animateNeighbors.update(get_world(next[0], next[1], self.grid_size, self.grid_dim))

        #if self.visualize:
        #    fig.canvas.flush_events()

        #self.parent = parent
        #self.g = g
        return parent, g

