

import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point


def extract_polygons(data):

    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [north-d_north, east-d_east, north+d_north, east+d_east]
        # TODO: Extract the 4 corners of the obstacle
        # 
        # NOTE: The order of the points matters since
        # `shapely` draws the sequentially from point to point.
        #
        # If the area of the polygon is 0 you've likely got a weird
        # order.
        corners = [(obstacle[0], obstacle[1]), (obstacle[0], obstacle[3]),
                   (obstacle[2], obstacle[3]), (obstacle[2], obstacle[1])]
        
        # TODO: Compute the height of the polygon
        height = alt + d_alt

        # TODO: Once you've defined corners, define polygons
        #p = Polygon(corners)
        p = Polygon(corners)
        #polygons.append((p, height))
        polygons.append((p, height))
    return polygons





# # Sampling 3D Points
# 
# Now that we have the extracted the polygons, we need to sample random 3D points. Currently we don't know suitable ranges for x, y, and z. Let's figure out the max and min values for each dimension.

# In[20]:

def sample(data, k):
    xmin = np.min(data[:, 0] - data[:, 3])
    xmax = np.max(data[:, 0] + data[:, 3])

    ymin = np.min(data[:, 1] - data[:, 4])
    ymax = np.max(data[:, 1] + data[:, 4])
    zmin = 0
    #zmax = 10
    #set to 0-10 for visualisation
   # zmin = np.min(data[:,2] - data[:,5])
    zmax = np.max(data[:,2] + data[:,5])
# Limit the z axis for the visualization
  
    num_samples = k

    xvals = np.random.uniform(xmin, xmax, num_samples)
    yvals = np.random.uniform(ymin, ymax, num_samples)
    zvals = np.random.uniform(zmin, zmax, num_samples)
    
    samples = list(zip(xvals, yvals, zvals))
    
    return samples



# ## Removing Points Colliding With Obstacles
# 
# Prior to remove a point we must determine whether it collides with any obstacle. Complete the `collides` function below. It should return `True` if the point collides with *any* obstacle and `False` if no collision is detected.

# In[27]:


def collides(polygons, point):   
    # TODO: Determine whether the point collides
    # with any obstacles.
    #print(point)
    for (p, height) in polygons:
        if p.contains(Point(point)) and height >= point[2]:
            return True
        
    return False


# Use `collides` for all points in the sample.

# In[28]:

def to_keep(samples, polygons):
    to_keep = []
    for point in samples:
        if not collides(polygons, point):
            to_keep.append(point)

    return to_keep

# In[29]:




# ## Points Visualization

# In[30]:






# [Solution](/notebooks/Random-Sampling-Solution.ipynb)

# ## Epilogue
# 
# You may have noticed removing points can be quite lengthy. In the implementation provided here we're naively checking to see if the point collides with each polygon when in reality it can only collide with one, the one that's closest to the point. The question then becomes 
# 
# "How do we efficiently find the closest polygon to the point?"
# 
# One such approach is to use a *[k-d tree](https://en.wikipedia.org/wiki/K-d_tree)*, a space-partitioning data structure which allows search queries in $O(log(n))$. The *k-d tree* achieves this by cutting the search space in half on each step of a query.
# 
# This would bring the total algorithm time down to $O(m * log(n))$ from $O(m*n)$.
# 
# The scikit-learn library has an efficient implementation [readily available](http://scikit-learn.org/stable/modules/generated/sklearn.neighbors.KDTree.html#sklearn.neighbors.KDTree).
