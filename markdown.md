# Rubrics and which part of code
### reading the lon0, lat0 
		' (lat0, lon0) = (37.792480, -122.397450) 
        global_home = (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0) ' 
		
### global_position was retrieved by using the udacidrone class methods 
		' global_position = (self._longitude, self._latitude, self._altitude) ' 
		after that, global_to_local function was used to convert to ECEF 
		
### start point and goal point  hardcoded; 
		grid_start = (316,  316)
		grid_goal = (750, 370)
		
### a_star algorithm and diagonal movements
		NORTH_EAST = (-1, 1, np.sqrt(2))
		NORTH_WEST = (-1, -1, np.sqrt(2))
		SOUTH_WEST = (1, -1, np.sqrt(2))
		SOUTH_EAST = (1, 1, np.sqrt(2))

		a_star was then ran and implemented to actually find the path we should traverse to reach goal
		results ( path and cost ) was then saved as variables; 
		
		path, cost = a_star(grid, heuristic_func, grid_start, grid_goal
		
### culling waypoints using collinearity
		pruned_path = prune_path(path)

	 waypoints then have been sent to autopilot 







# motion-planning vs backyard-flyer-solution
	The difference here between both of them is, in backyard_flyer we manually input the path vehicle takes; 
	we do not manually control the vehicle though. however in motion_planning, we just tell the vehicle
	that we want to get from X1 to X2 and the vehicle does all the rest of job, using the planning utils
	this is what we have now as the planning state, the state all of these utilities are ran at. 
##Details
	    1. The Create Grid function makes like a map, that our drone can understand and operate according to, without
	  this the drone doesn't really recognise the environment and surrounding. 
	    2. global-to-local function what we use to convert from coordinate systems the GPS and Sensors use
	    to the coordinate system the drone uses
	    3. a_star function; after we have successfully created the grid, the map that our drone operates on, adding obstacles
	     and other neccessary environmental objects, this function plans and calculate the path we should traverse 
	     to reach the goal we want. 
	  
##the plan path works as follows; 
		1. set safety distance and target altitude
		2. load actual map data ( the csv file ..etc )
		3. retrieve global-home position and global-position
		4. convert to local position using global-to-local function
		5. create the grid containing the map data
		6. implement the a_star function which will plan the path and set of actions
			needed to reach the goal
		7. implement the prune_path method, which reduces the waypoints; 
			waypoints are points along the path we traverse. withot actually affect-ing final result ( reaching the goal state ) 
		8. sends waypoints to the autopilot and start the transition phase