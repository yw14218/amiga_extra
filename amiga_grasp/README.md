# amiga_manip

A grasp generator package based on object detection, grasp pose detection and dex-net.

## Structure
`conifg`: configuration files for GPD and dex-net with Amiga-specific settings
`data` captured intermediate real-world data (point clouds as well as RGB-D images)


## Usage

Make sure you have installed GPD and Dex-Net following [moveit deep grasp_demo](https://github.com/PickNikRobotics/deep_grasp_demo)

You will need our [fork](https://github.com/yw14218/deep_grasp_demo) in order to use deep grasp with Amiga

## 3 DoF Bounding-box Grasp (known orientation + 3D location from object detection)

```
roslaunch amiga_grasp obj_det_grasp.launch
```

## Grasp Pose Detection

### How does it work?

The pipeline is as follows:

1. launch the gpd and cloud-saving server
2. capture a point cloud data from cameras or perception (object/region point cloud)
3. cloud processsing / removing ground using Open3D
4. send the processed cloud to the gpd server for a 6 DoF grasp inference
5. translate to Amiga's grasp pose in base_link
6. send it to Amiga_manip's grasp executor to plan a grasp

### usage 

`launch/grasp_generator/gpd.launch` does step 1
```
roslaunch amiga_grasp gpd.launch
```
This launches the gpd server as well as a utility cloud-saving server.
Unused arguments are commented on within the NOT USED tags.

`src/grasp_generator/gpd_grasp.py` does steps 2 - 5

The communication is via looking up point clouds in the data/tmp folder

This generates a grasp pose in Amiga's base link and publishes a TF

## Dex-Net

### How does it work?

The pipeline is as follows:

1. launch the dexnet server
2. capture RGB-D images from cameras or perception (object/region cropped images)
3. send the RGB-D images to the dexnet server for grasp inference
4. translate to Amiga's grasp pose in base_link
5. send it to Amiga_manip's grasp executor to plan a grasp


### usage 

`launch/grasp_generator/dexnet.launch` does step 1
Besides model configuration files you will need a policy script as in [here](https://github.com/yw14218/deep_grasp_demo/blob/master/moveit_task_constructor_dexnet/scripts/grasp_detector)

`scripts/collect_image_for_dex.py` does step 2

`amiga_grasp/src/grasp_generator/dexnet_grasp.py ` does step 3 - 4

The communication is via looking up images in the data/tmp folder and using NumPy files are proven to be more efficient.

### 4 DoF

`rasp_generator/4DoFdexnet_grasp.py`

The idea is to use only 1D rotation from dex-net's inference result (the original one is 6D grasp pose)

## Demos




