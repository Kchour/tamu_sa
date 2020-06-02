#!/usr/bin/env python3
''' Independent '''
from search.search_utils import PriorityQueue
import matplotlib.pyplot as plt
import numpy as np
import time
import networkx as nx

''' Dependecies '''
from animation.animation import Animate


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
    def __init__(self, graph, start, goal, h_type, visualize=False):
        Search.__init__(self, graph, start, goal)
        self.h_type = h_type
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
            # 50 millisecond sleep
            self.animateCurrent = Animate(number=1, xlim=xlim, ylim=ylim, gridSize=1,linewidth=5, markerType='xc', markerSize=10, sleep=0.000, order=10)
            self.animateNeighbors = Animate(number=1, xlim=xlim, ylim=ylim, gridSize=1,linewidth=5, markerType='o', markerSize=5, sleep=0.000, order=-1)
            self.animatePath = Animate(number=1, xlim=xlim, ylim=ylim, gridSize=1,linewidth=5, markerType='o', markerSize=5, sleep=0.000, order=-1)


    ''' heuristic function '''
    def heuristic(self, a, b, type_='manhattan'):
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
    
    ''' perform one iteration of algorithm '''
    def iterate_algorithm(self):
        ''' Usage:
            - call to step through the algorithm for 1 iteration
            
            Returns:
            - a linked list, 'parent'
            - and hash table of nodes and associated min cost, 'g'
        '''
        frontier = self.frontier
        parent = self.parent
        g = self.g  #basically the closed list

        if not frontier.empty():
            current = frontier.get()  # update current to be the item with best priority

            if self.visualize:
                # Update plot with visuals
                self.animateCurrent.update(current)
            # Exit if we reached our goal
            if current == self.goal:
                return parent, g, current
            # expand current node and check neighbors
            for next in self.graph.neighbors(current):
                g_next = g[current] + self.graph.cost(current, next)
                # if next location not in CLOSED LIST or its cost is less than before
                # Newer implementation
                if next not in g or g_next < g[next]:
                    g[next] = g_next
                    if self.h_type == 'zero' or self.goal==None:
                        priority = g_next 
                    else:
                        priority = g_next + self.heuristic(self.goal, next, self.h_type)
                    frontier.put(next, priority)
                    parent[next] = current
                   
                    if self.visualize:
                        self.animateNeighbors.update(next)
        else:
            current = None

        return parent, g, frontier.elements, current    #current is the latest to be put in closed list

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
                        priority = g_next + self.heuristic(self.goal, next, self.h_type)
                    frontier.put(next, priority)
                    parent[next] = current

                    if self.visualize:
                        self.animateNeighbors.update(next)

        #if self.visualize:
        #    fig.canvas.flush_events()

        #self.parent = parent
        #self.g = g
        return parent, g

''' Generic Class for depth first search
    Modified to detect cycles
'''
class DepthFirstSearch(Search):
    def __init__(self, graph, start, visualize=False):
        Search.__init__(self,graph, start, None)
        self.visualize = visualize

        self.frontier = PriorityQueue()
        self.frontier.put(self.start, 0)
        self.parent = {}
        self.parent[self.start] = None
        self.closedList = []    #leaf nodes

        if visualize:
            self.fig, self.ax = plt.subplots(1,1, num=10)
            plt.show(block=False)
            self.nxG = nx.Graph()
            self.pos = []

    def use_algorithm(self):
        frontier = self.frontier
        parent = self.parent
        g_next = 0
        visited = []
        visitedNodes = []
        isCycle = False
        
        while not frontier.empty():
            current = frontier.get()
            #ensures we expand unvisited nodes!                 
            visited.append(current)
            neighbors = self.graph.neighbors(current)
            for next in neighbors:
                g_next -= 1
                if next not in visited:
                    priority = g_next
                    frontier.put(next, priority)
                    parent[next] = current
                    # print(current, next, visited)

                    # record nodes visited
                    #visitedNodes1.append(current)
                    visitedNodes.append(next)

                    if self.visualize:
                        self.nxG.add_edge(current, next)
                        if not self.pos:
                            self.pos = nx.spring_layout(self.nxG)
                        else:
                            self.pos = nx.spring_layout(self.nxG, pos =self.pos, fixed=visited)
                        self.ax.clear()
                        nx.draw(self.nxG, ax=self.ax, pos=self.pos, edge_color="blue", with_labels = True)
                        self.ax.set_xticks([])
                        self.ax.set_yticks([])
                        plt.pause(0.0001)
        # We can check for duplicates in here
        if len(visitedNodes) != len(set(visitedNodes)):
           isCycle = True
        return isCycle


class DjikstraSearch():
    def __init__(self):
        print("WIP")

