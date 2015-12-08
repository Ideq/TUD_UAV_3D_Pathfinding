# Generator of 3-Dimensional vector field for path following
This repository contains python helper files which allows the calculation and display of 3D paths and vector fields for using in robot path following.

## Requirenments:

Tested in Ubuntu 12.04 and 14.04 with Python 2.7

## Installation

1. Install Mayavi (Scientific visualization package for 2D and 3D data

        sudo apt-get install mayavi2
2. Clone the repo.

        git clone git@github.com:raultron/3d_path_vector_field.git
3. Execute the file "Example.py" in python. If everything is working you should see a Mayavi windows with a visualization of the Path, the 3D vector field and the streamlines.


## Documentation

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
