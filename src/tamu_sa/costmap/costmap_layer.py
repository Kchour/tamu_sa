#!/usr/bin/env python

import numpy as np
import pdb

'''
{name: numpy array}
'''
class CostMap: 
    def __init__(self):    
        self.layers= {}

    def add_layer(self, grid=None, name=""):
        assert  not name in self.layers, '"{}"  ALREADY EXISTS'.format(name)
        self.layers[name] = grid
    
    # THIS FUCNTION MAY BE USELESS
    def update_layer(self, grid, name):
        assert name in self.layers, '"{}" NOT FOUND, ADD IT FIRST'.format(name)
        self.layers[name] = grid 

    def does_layer_exist(self, name):
        ''' helper function to only add layer once 
            name := a tuple or list of names ('name1', name2')
        '''
        if not isinstance(name,list) and not isinstance(name, tuple):
            name = [name]
        #return name in self.layers
        return  all(n in self.layers for n in name)

    def return_total_map(self):  
        # Convert self.layers to a 3d array
        array_3d = np.dstack(list(self.layers.values()))
        return np.sum(array_3d, axis=2)

    def delete_layer(self, name):
        del self.layers[name]
        print('Deleted "{}"'.format(name))


           
        

            

  
        