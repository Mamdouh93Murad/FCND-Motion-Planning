
# coding: utf-8

# ### Geodetic to NED  

# In[37]:


# First import the utm and numpy packages
import utm
import numpy


# To convert a GPS position (_longitude_, _latitude_, _altitude_) to a local position (_north_, _east_, _down_) you need to define a global home position as the origin of your NED coordinate frame. In general this might be the position your vehicle is in when the motors are armed, or some other home base position. You first task is to define a function to convert from global position to a local position using the `utm`. To do this fill in the TODO's below!

# In[38]:


def global_to_local(global_position, global_home):
    
    # TODO: Get easting and northing of global_home
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(global_home[1], global_home[0])
    # TODO: Get easting and northing of global_position
    (east, north, zone_number, zone_letter) = utm.from_latlon(global_position[1], global_position[0])
    # TODO: Create local_position from global and home positions                                     
    local_position = numpy.array([north - north_home, east - east_home, -(global_position[2] - global_home[2])])
    
    return local_position


# ### NED to Geodetic
# Now try converting a local position (_north_, _east_, _down_) relative to the home position to a global position (_long_, _lat_, _up_).

# In[39]:


def local_to_global(local_position, global_home):
    
    # TODO: get easting, northing, zone letter and number of global_home
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(global_home[1], global_home[0])
    # TODO: get (lat, lon) from local_position and converted global_home
    (lat, lon) = utm.to_latlon(east_home + local_position[1], north_home + local_position[0], zone_number, zone_letter)
    # TODO: Create global_position of (lat, lon, alt)
    
                               
    global_position = numpy.array([lon, lat, -(local_position[2] - global_home[2])])
    
    return global_position



