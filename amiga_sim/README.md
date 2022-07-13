# amiga_sim

A simulation toolbox for Amiga based on Gazebo.


Feature | Perception | Manipulation | Grasping | Navigation
--- | --- | --- | --- |--- |
Plugin | L515 camera :heavy_check_mark: | ZED 2 camera :x: | Grasp Plugin :heavy_check_mark: | Hokuyo :heavy_check_mark:
Teleop | :x: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
Advanced | :heavy_check_mark: | [amiga_manipulation](https://github.com/yw14218/amiga_extra/tree/main/amiga_manipulation) :heavy_check_mark: | [amiga_grasp](https://github.com/yw14218/amiga_extra/tree/main/amiga_grasp) :heavy_check_mark:| gmapping & amcl :heavy_check_mark:


## structure
* `/amiga_description` directory to Amiga's URDF with gazebo plugins added
* `/calibration_sim` camera intrinsic & hand-eye calibration features integrated
* `/config` rviz and ROS navigation stack configuration files
* `/urdf` some example models we made 
* `/worlds` some example worlds we made (for large worlds you will need a powerful GPU)

## running

```
roslaunch amiga_sim grasp_sim.launch
```

Please see `launch/grasp_sim.launch` for how to switching amongst different world
Please see [husky_navigation](http://wiki.ros.org/husky_navigation/Tutorials/Husky%20AMCL%20Demo) for how to use navigation

![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_sim/media/kitchen_sim.png)
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_sim/media/grasp_place_sim.gif)
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_sim/media/pour_sim.gif)
![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_sim/media/pick_handle.gif)
