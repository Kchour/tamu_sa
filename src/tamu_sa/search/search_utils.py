''' search utils '''

import collections 
import numpy as np

''' custom data structures for search algorithms. Import methods are:
    :empty: True if empty, otherwise False
    :put:   adds a new node to the data structure
    :get:   returns a node from data structure (based on some criteria)
'''

# Template class for defining your own data structures for using search algorithm
class CustomQueue:
    def __init__(self):
        ''' self.elements = '''
    
    def empty(self):
        ''' return len(self.elements) == 0 '''
    
    def put(self, x):
        ''' define stuff '''
    
    def get(self):
        ''' define stuff '''

class Queue:
    ''' First In First Out '''
    def __init__(self):
        self.elements = collections.deque()

    def empty(self):
        return len(self.elements) == 0

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()


# class PriorityQueue:
#     def __init__(self):
#         self.elements = {}

#     def empty(self):
#         return len(self.elements) == 0

#     def put(self, item, priority):
#         self.elements[item] = priority

#     def get(self):
#         # Iterate through dictionary to find the item with the best priority
#         best_item, best_priority = None, None
#         for item, priority in self.elements.items():
#             if best_priority is None or priority < best_priority:
#                 best_item, best_priority = item, priority

#         # Remove the best item from the OPEN LIST
#         del self.elements[best_item]

#         # return
#         return best_item


''' Priority Queue with heapq. Thanks RedBlob '''
import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]


''' post-processing utility functions'''

def reconstruct_path(parents, start, goal, order='forward'):
    # parents are now in index
    
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = parents[current]
    path.append(start)
    if order == 'forward':
        path.reverse()

    return np.array(path)

def path_length(path):
    # compute costs
    diff_ = np.diff(path, axis=0)
    hyp_ = np.hypot(diff_[:,0], diff_[:,1])
    dist_ = sum(hyp_)
    return dist_

# ''' example heuristic functions '''

# def heuristic(self, a, b, type_='manhattan'):
#     (x1, y1) = a
#     (x2, y2) = b
#     if type_ == 'manhattan':
#         return abs(x1 - x2) + abs(y1 - y2)
#     elif type_ == 'euclidean':
#         v = [x2 - x1, y2 - y1]
#         return np.hypot(v[0], v[1])
#     elif type_ == 'diagonal_uniform':
#         return max(abs(x1 - x2), abs(y1 - y2))
#     elif type_ == 'diagonal_nonuniform':
#         dmax = max(abs(x1 - x2), abs(y1 - y2))
#         dmin = min(abs(x1 - x2), abs(y1 - y2))
#         return 1.414*dmin + (dmax - dmin)
    
# =================== Pairwise L2 Distance ====================
''' TODO Compute pairwise distance between two sets of vectors '''
def compute_l2(A, B):
    '''
    <A> is m x p array (each row is a p-vector)
    <B> is m x n array ()
    returns <C> :=   
    '''
    print("wip")