# amiga_plane fitter

A C++ node used to extract/remove ALL planes in a given point cloud based on pcl library.

## usage

```
roslaunch amiga_obstacle_avoid amiga_plane_fitter.launch
```

The node keeps applying RANSAC until a min percentage of points are left. See `amiga_plane_fitter.launch` for meanings of each parameter.

