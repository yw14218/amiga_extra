# amiga_plane fitter

A C++ node used to extract/remove ALL planes in a given point cloud based on pcl library.

## usage

```
roslaunch amiga_obstacle_avoid amiga_plane_fitter.launch
```

The node keeps applying RANSAC until a min percentage of points are left. See `amiga_plane_fitter.launch` for meanings of each parameter.

![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_obstacle_avoid/media/15.png)
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_obstacle_avoid/media/16.png)
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_obstacle_avoid/media/67.png)
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_obstacle_avoid/media/68.png)
