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
				######output
				######code
			#####angle_calculation
				######input
				######output
				######code
			#####show_path
				######input
				######output
				######code
			#####followPath
				######input
				######output
				######code

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
