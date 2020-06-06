''' custom graph-like objects defined here. import class methods are
    :SquareGrid: 
        :neighbors(v):  given tuple (x,y), returns neighbors
        :cost(v1, v2):  given two tuples (v1, v2), returns cost of edge

    :MyGraph:   User can define a generic graph, by giving a edge list
        :neighbors(v):  similar to above, but v is generic here
        :cost(v1,v2):   similar to above, but v1,v2 is generic
'''

import numpy as np

from .grid_utils import init_grid
from .grid_utils import get_index
from .grid_utils import get_world

import pdb
class SquareGrid:
    def __init__(self, grid, grid_dim, grid_size, type_=4):
        self.xwidth = grid_dim[1] - grid_dim[0]
        self.yheight = grid_dim[3] - grid_dim[2]
        #Initialize grid if nothing is passed in
        if np.size(grid) != 0:
            self.grid = grid
        else:
            self.grid = init_grid(grid_dim, grid_size, 0)
        self.grid_dim = grid_dim
        self.grid_size = grid_size
        self.neighbor_type = type_

    def in_bounds(self, ind, type_='map'):
        if type_ == 'world':
            (x, y) = ind
            return self.grid_dim[0] <= x <= self.grid_dim[1] and self.grid_dim[2] <= y <= self.grid_dim[3]
        else:
            # grid indices
            (indx, indy) = ind
            xcells = int(np.ceil((self.xwidth + 1) / self.grid_size))
            ycells = int(np.ceil((self.yheight + 1) / self.grid_size))
            return 0 <= indx <= xcells and 0 <= indy <= ycells

    def not_obstacles(self, ind, type_='map'):
        if type_ == 'map':
            (indx, indy) = ind
            # may have issues in the future...
            return self.grid[indy, indx] == 0
        else:
            # convert world to ind first
            (indx, indy) = get_index(ind[0], ind[1], self.grid_size, self.grid_dim)
            return self.grid[indy, indx] == 0

    def neighbors(self, xxx_todo_changeme):
        ''' modified to do less converting, stay in world frame'''
        # Convert world coordinates to indices
        # (x, y) = xxx_todo_changeme
        # (indx, indy) = get_index(x, y, self.grid_size, self.grid_dim)
        # (indx, indy) = xxx_todo_changeme
        # if self.neighbor_type == 4:
        #     results = [(indx + 1, indy), (indx, indy - 1),
        #                (indx - 1, indy), (indx, indy + 1)]
        # elif self.neighbor_type == 8:
        #     results = [(indx + 1, indy), (indx, indy - 1),
        #                (indx - 1, indy), (indx, indy + 1),
        #                (indx + 1, indy + 1), (indx + 1, indy - 1),
        #                (indx - 1, indy - 1), (indx - 1, indy + 1)]

        (x, y) = xxx_todo_changeme
        if self.neighbor_type == 4:
            results = [(x + self.grid_size, y), (x, y - self.grid_size),
                       (x - self.grid_size, y), (x, y + self.grid_size)]
        elif self.neighbor_type == 8:
            results = [(x + self.grid_size, y), (x, y - self.grid_size),
                       (x - self.grid_size, y), (x, y + self.grid_size),
                       (x + self.grid_size, y + self.grid_size), (x + self.grid_size, y - self.grid_size),
                       (x - self.grid_size, y - self.grid_size), (x - self.grid_size, y + self.grid_size)]


        # Only return coordinates that are in range
        results = filter(lambda x: self.in_bounds(x, type_='world'), results)

        # Only return coordinates that are not obstacles
        results = filter(lambda x: self.not_obstacles(x, type_='world'), results)

        ## convert results to world coordinates
        # results = map(
        #     lambda v: get_world(
        #         v[0],
        #         v[1],
        #         self.grid_size,
        #         self.grid_dim),
        #     results)
        return results

    # Cost of moving from one node to another (edge cost)
    def cost(self, from_node, to_node):
        a = from_node
        b = to_node
        v = (b[0] - a[0], b[1] - a[1])
        return np.hypot(v[0], v[1])

class MyGraph:
    def __init__(self, edge_dict, allow_cycles=True):
        '''edge_dict: is dict containing all edges and weights
                      {('v1', 'v2'): 5.0}

		edges = [('v1', 'v2'), ('v1','v3')]
		weights = [0.5, 0.4]

		g = MyGraph()
        '''
        # TODO Consider creating an adjaceny list, to make finding neighbors faster
        self.edge_dict = edge_dict 

    def update(self, edge_dict):
        self.edge_dict.update(edge_dict)
    
    def remove(self, edge_dict):
        for e in edge_dict.keys():
            self.edge_dict.pop(e)

    '''neighbors returns a list [] '''
    def neighbors(self, v):
        # Create a list of edges containing v
        edgeList = [key for key, val in self.edge_dict.items() if v in key]


        # Go through each tuple in test2 and select neighbors 
        neighs  = [node for edge in edgeList for node in edge if node != v]

        # remove duplicates by creating a dictionary
        neighs = list(dict.fromkeys(neighs))
        
        return neighs
    
    ''' cost returns edge weight '''
    def cost(self, from_node, to_node):
        weight = self.edge_dict[(from_node, to_node)]
        return weight
