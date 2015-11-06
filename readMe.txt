This programm is tested for V-REP in combination spyder.

Mapdata:
The file "arr_0000.npy" contains the mapdata, if you dont want to run the mapgeneration code for "testroom.ttt" open this array in spyder und rename it to "arr". To create the mapdata it takes like 30min, this needs to be implemented in another way in the future, then it will be a lot faster.

Simulation scene:
"testroom111.ttt"

Programm code:
The code is located in "vrep_mapdata_python.py" it contains all steps from mapgeneration->path finding->path smoothing

Programm execution:
To run the pathfinding programm first you need to start the simulation in V-REP, then its necessary to run the whole code in "vrep_mapdata_python.py", if you load the mapdata from the file before runing its possible to comment lines 113-148, like they are at the current version.
