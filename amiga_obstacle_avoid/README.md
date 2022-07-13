# amiga_plane fitter

A C++ node used to extract/remove ALL planes in a given point cloud based on pcl library. The result pooint cloud in `/published amiga_plane_fitter` topic
with at least 15 HZ. 

## usage

Currently work for l515 and ZED 2 cameras
```
roslaunch amiga_obstacle_avoid amiga_plane_fitter.launch
```

The node keeps applying RANSAC until a min percentage of points are left. See `amiga_plane_fitter.launch` for meanings of each parameter.

## example 
Used in conjunction with the octomap library

### simulation
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_obstacle_avoid/media/67.png)

### real world (both table/chair and ground are planes)
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_obstacle_avoid/media/15.png)
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_obstacle_avoid/media/68.png)

### after adding collidable object regions into the scene
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_obstacle_avoid/media/16.png)
