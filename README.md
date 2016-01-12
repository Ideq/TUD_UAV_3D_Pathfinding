#3D pathgeneration and pathfollowing for UAV in VREP 
This repository contains python code, that can be ran in combination with the Roboter Simulation Platform VREP. We used this Platform to simulate Pathfinding, Pathsmoothing und Pathfollowing for an UAV in some different areas(one small test area and a bigger area showing 2 buildings of our univesity)

## Requirenments:

Tested in Windows 8.1 and Mac OS with Python 2.7 in Spyder

## Installation

1. Install VREP

2. Clone the repo

3. Start the simulation of an area

4. Run the UAV_main.py to start the mapdatageneration, pathgeneration and pathfollowing algorythm

	###Step 1:
	
	Generates an array with the mapdata for the current scene in VREP
	
	Step 2:
	
	Reads the starting position and the goal from the scene
	
	Step 3:
	
	Finds and interpolates a Path
	
	Step 4:
	
	Starts the pathfollowing
	
	Step 5:
	
	After the goal is reached, it shows the calculated path in comparison to the real path the UAV was flying in 2D.
## Documentation

	###UAV_main.py
		####imports
		vrep.py, for the API-functions 
		UAV_mapgen.py, for the mapdata-functions
		UAV_pathfinding.py, for the pathfinding-functions
		UAV_VREP.py, for the functions to communicate with V-REP
		numpy.py, for some math-functions
        matplotlib.pyplot and mpl_toolkits.mplot3d, for plotting
		####code
		Contains the main-script of our project, here are all other parts combined. First a connection to V-REP is established, 
		then the mapdata is generated or loaded if she already exists. Next step is to read the start and the goal position from V-REP. 
		Thats all information, which is needed for the pathfinding and generation. After the pathgeneration is finished a signal is send to V-REP,
		so the path can be drawn in the simulation by a part in the UAV-LUA-script.
		Thats the point, when the path-following can be started. V-REP is ran in the synchronous-mode, to make sure for every simulation step the 
		information/direction/orientation is updated.
		After the path-following is finished a plot is created where the real path which the UAV was flying is compared with the calculated path in 2D.
		The xy-plane is shown, the calculated path is shown as line, the real path is shown by the points where the UAV was before the next simulation step was executed.
	
	###UAV_VREP.py
		####imports
		vrep.py, for the Connection with the Simulator
		numpy.py, for the arrays and some other mathematical operations
		time.py, for the sleep-function
		math.py, for some math-functions
		pathfollowing.py, for the function which returns the direction the UAV need to move to follow the path
		####functions
			#####getPosition
				######input
				clientID, integer, needed to execute API-functions
				object_name, string, the name of the object, which position you want to get
				######output
				position, array with the 3 coordinates x,y,z in meters
				######code
				First the object handle is returned from V-REP, afterwards its possible to get the position from V-REP.
			#####angle_calculation
				######input
				a and b, arrays(1x3), 3D-vectors
				######output
				angle, float, the angle between the two vectors in radiant, always returns the small angle
				######code
				Simple angle calculation with the scalar-product and the arcus-cosinus using functions from math.py
			#####show_path
				######input
				path, array, contains 1 array for each coordinate, the length of these arrays depends on the length of the path
				clientID, integer, needed to execute API-functions
				######output
				no return value, but this function creates string signals with the coordinate arrays, which are read from the UAV-LUA-script
				######code
				After the path-array is divided, the 3 signals are created.
			#####followPath
				######input
				clientID, integer, needed to execute API-functions
				path, array, contains 1 array for each coordinate, the length of these arrays depends on the length of the path
				goal, tupel, contains the 3 coordinates of the goal-position
				######output
				plot-data, array, contains the position of the UAV in each simulation step
				######code
				Depending on the current position of the UAV and the path a direction for the UAV is calculated and converted into velocities, 
				which are given to the UAV, by creating string signals, that are checked by the UAV-LUA-script. After the calculation is finished 
				the next simulation-step is tiggered. This algorythm is repeated, till the goal-position is reached.
				Afterwards the signals are cleaned up and some plot-data is returned to the main-script.
				





        ###pathfollowing.py
              	        ####imports
	     	                 numpy.py, for the arrays and some other mathematical operations 
	     	        ####functions
	     	                 ######input 
	     	                 position, an array with the 3 coordinates x,y,z in meters which we get after pathfinding
	     	                 path, an array, contains 1 array for each coordinate, the length of these arrays depends on the length of the path
	                         ######output
	                         v_result, a vector which dicides how does the UAV go into the path. 
	                         v_tangent_nor, the tangent vector after the normalization
	                         p_near, the nearest point in the path to the current position
	                         distance, the distance between the nearest point and the current position
	                ######code
	                if the distance is not too big, the v_result is combined by two vectors: v_approx_nor and v_tangent_nor.
The v_approx_nor is the vector which we get from the difference between the current position and the nearest point. We also did the normalization.
                       if the distance is very big, bigger than 0.2 in our case, the v_result comes just from the v_approx. It makes the pathfollowing faster.


        ###Scene: hexagon_neu.ttt
                         Abstract: In the scene we have the S311 building which contains the walls and the windows. The goal_new object is the goal which we want the UAV to fly to. You can also move the goal. The UAV script is the main part of the scene. It can control the UAV and also draw the path which we calculated. We learned from our betreuer Raul's script, which will also be discribed on the following. 
                         ##### UAV code
                         unfinished
                         
                         
        ### Rauls UAV code
                         unfinished





* The function **path_3d(pos, loop)** included in the file "PathGenerationCubic.py" takes as an argument an array of 3D points (pos) where the path should go through and a boolean (loop) which defines if the trayectory is closed or open. The return of the function is an array for each coordinate of the path and the time (path_x, path_y, path_z and path_t).

* The function **calc_vec_field_fast(path_x, path_y, path_z, cube_size, cube_step)** included in the file "FieldGeneration.py" defines a 3D matrix CUBE for the field of size cube_size and with step size cube_step. For each point inside this cube it calculate the best aproximation vector to the path using the methodology explained in the paper: TODO. The return of the function is the calculated vector field and some helper grids for displaying this field in mayavi.



             
                   
                   
                   

## Example

In the file "example.py" we define 3 points in a closed loop configuration:

```
Pos0 = array([4, 8, 3])
Pos1 = array([14, 12, 17])
Pos2 = array([14, 4, 17])
Loop = True
Pos = array([Pos0, Pos1, Pos2])

path_x, path_y, path_z, path_t = path_3d(Pos, Loop)
```

and then calculate the vector field with:

```
vector_field_3D, Xc, Yc, Zc = calc_vec_field_fast(X, Y, Z, 20, 1)
```

Finally this script uses mlab (Mayavi) to show the resulting path, the vector field and the striplines to observe the possible paths from every point in space.

To create a different path it is only necessary to modify the example file (example.py) with diferent paths and cube sizes.
