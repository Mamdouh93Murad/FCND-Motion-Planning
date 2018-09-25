
# coding: utf-8

# # Probabilistic Roadmap
# 
# 
# In this notebook you'll expand on previous random sampling exercises by creating a graph from the points and running A*.
# 
# 1. Load the obstacle map data
# 2. Sample nodes (use KDTrees here)
# 3. Connect nodes (use KDTrees here)
# 4. Visualize graph
# 5. Define heuristic
# 6. Define search method
# 7. Execute and visualize
# 
# We'll load the data for you and provide a template for visualization.

# In[1]:


# Again, ugly but we need the latest version of networkx!
# This sometimes fails for unknown reasons, please just 
# "reset and clear output" from the "Kernel" menu above 
# and try again!
import sys
#!{sys.executable} -m pip install -I networkx==2.1
import pkg_resources
#pkg_resources.require("networkx==2.1")
import networkx as nx



import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue
from sklearn.neighbors import KDTree
import numpy.linalg as LA
from sampling import sample


# ## Step 3 - Connect Nodes
# 
# Now we have to connect the nodes. There are many ways they might be done, it's completely up to you. The only restriction being no edge connecting two nodes may pass through an obstacle.
# 
# NOTE: You can use `LineString()` from the `shapely` library to create a line. Additionally, `shapely` geometry objects have a method `.crosses` which return `True` if the geometries cross paths, for instance your `LineString()` with an obstacle `Polygon()`!

# In[74]:


# TODO: connect nodes
# Suggested method
       # 1) write a method "can_connect()" that:
        # casts two points as a shapely LineString() object
        # tests for collision with a shapely Polygon() object
        # returns True if connection is possible, False otherwise
def can_connect(p1, p2):
    line = LineString([p1,p2])
    for p in polygons:
         if p.crosses(line) and p.height >= min(p1[2],p2[2]):
            return False
    return True
    # 2) cast nodes into a graph called "g" using networkx
     # 3) write a method "create_graph()" that:
        # defines a networkx graph as g = Graph()
        # defines a tree = KDTree(nodes)
        # test for connectivity between each node and 
            # k of it's nearest neighbors
        # if nodes are connectable, add an edge to graph
    # Iterate through all candidate nodes!
def create_graph(nodes, k):
    graph = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes: 
        idx = tree.query([n1], k, return_distance = False)[0]
        for i in idx:
            n2 = nodes[i]
            if n2 == n1:
                continue
            if can_connect(n1,n2):
                graph.add_edge(n1,n2,weight=1)
    return graph
            


# In[109]:


#run the create_graph function on attributes to actually construct the graph


# ## Step 4 - Visualize Graph

# In[110]:


def heuristic(n1, n2):
    cost = LA.norm(np.array(n2) - np.array(n1))
    return cost


# ## Step 6 - Complete A*

# In[136]:


def a_star_graph(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    path = []
    queue = PriorityQueue()
    queue.put((0,start))
    visited = set(start)
    
    branch = {}
    found = False
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]
        
        if current_node == goal:
            found = True
            print('Path Found!')
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
            if next_node not in visited:
                visited.add((next_node))
                queue.put((new_cost, next_node))
                branch[next_node] = (new_cost, current_node)
    # TODO: complete
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
            
    return path[::-1], path_cost



def pair_points(path):
    path_pairs = zip(path[:-1], path[1:])
    return path_pairs


def plot_graph(graph, path_pairs):
# draw nodes
    for n1 in graph.nodes:
        plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
    
# draw edges
    for (n1, n2) in graph.edges:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black')
    
# TODO: add code to visualize the path
    for (n1, n2) in path_pairs:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')


    plt.xlabel('NORTH')
    plt.ylabel('EAST')

    plt.show()



