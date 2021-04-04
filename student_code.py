"""
A * algorithm using minheap

"""

# Libraries
import heapq


#------------------------------------------------
# Helper class to implement and override priority queues
#------------------------------------------------
class GraphNode:
    def __init__(self, val, heuristic=0):
        """
        val: represents the value of the node
        heuristic: represnts that heuristic on which the minheap will operate
        """
        self.val = val
        self.heuristic = heuristic
    
    # overriding methods of the heapq
    def __lt__(self, node):
        return self.heuristic < node.heuristic
    
    def __eq__(self, node):
        return self.heuristic == node.heuristic

def calc_distance(x1, x2, y1, y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5

#------------------------------------------------
# function that implements A* Route algorithm
#------------------------------------------------

def shortest_path(M,start,goal):
    
    # defining start node
    startnode = GraphNode(val=start) # start node has the min value in the heapq
    
    # Extracting vertices and edges
    vertices, edges = M.intersections, M.roads
    
    # using priority queue to define the frontier
    frontier = list()
    heapq.heapify(frontier)
    
    # initializing frontier with the start node
    heapq.heappush(frontier, startnode)
    
    # dict to keep track of cost of path travelled
    explored = dict()
    explored[start] = 0
    # dict to keep track of path
    path_dict = dict()
    path_dict[start] = -1 # storing -1 as place holder for start 
    
    # run loop while the frontier has elements
    while frontier:
        
        # popping minimum heuristic from heapq
        current_vertex = heapq.heappop(frontier)
        
        # break the loop if goal is reached
        if current_vertex.val == goal:
            print("Reached goal")
            break
        # else explore the neighbouring intersections
        else:
            neighbours = edges[current_vertex.val]
            if len(neighbours) == 0:
                continue
            
            # search through all neighbours to find min distance
            for n in neighbours:
                x, y = vertices[current_vertex.val][0], vertices[current_vertex.val][1]
                x_adj, y_adj = vertices[n][0], vertices[n][1]
                
                # euclidian distance formula used
                # https://en.wikipedia.org/wiki/Euclidean_distance
                cost = explored.get(current_vertex.val, 0) + calc_distance(x, x_adj, y, y_adj)
                #cost = explored.get(current_vertex.val, 0) + ((x - x_adj)**2 + (y - y_adj)**2) ** 0.5
                
                # if current cost is less, then update the path cost dict
                if n not in explored.keys() or explored[n] > cost:
                    
                    path_dict[n] = current_vertex.val
                    explored[n] = cost
                    
                    # calculate the new heuristic
                    heuristic = cost + calc_distance(vertices[goal][0], x_adj, vertices[goal][1], y_adj)
                    #heuristic = cost + ((vertices[goal][0] - x_adj)**2 + (vertices[goal][1] - y_adj)**2) ** 0.5
                    
                    #Push the next_node onto the Priority Queue
                    heapq.heappush(frontier, GraphNode(n, heuristic))
    
    # define path list to generate the path traversed
    path = list()
    element = goal
    
    while True:
        if start == element:
            break
        path.append(element)
        element = path_dict[element]

    # reversing the path list to create path from start to goal
    return [start] + path[::-1]
                    