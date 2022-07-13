# amiga_manip

A hierarchical manipulation server for Amiga based on MoveIt. 

## core concept

* Complex manipulation tasks can be described as a combination of simpler ones. The purpose of this server is to identify and provide several useful services to Amiga developers such that they can concentrate on investigating how to use perception to generate a desired online trajectory and action plan.

* A single manipulation server is more organised, extensible and trackable than spamming scripts for each task.

* Services can be used in a script via proxies. Some examples can be found in the `scripts` folder

* Compatible with Amiga simulation. Set the simulation argument to true if in use.
 
## low-level services 

### go to a pre-defined pose (update in the SRDF file for more)

```
/amiga/offline_manipulation/go_to_grasp_home_pose
/amiga/offline_manipulation/go_grasp_front_pose
/amiga/offline_manipulation/go_to_zero_pose
/amiga/offline_manipulation/go_to_home_pose
/amiga/offline_manipulation/go_to_inspect1_pose
/amiga/offline_manipulation/go_to_inspect2_pose
```
example usage:
```
rosservice call /amiga/offline_manipulation/go_to_grasp_home_pose
```


### plan the end-effector to a given pose in a given frame (transformations performed automatically)

```
/amiga/offline_manipulation/plan_to_pose
```

example usage:
```
rosservice call /amiga/offline_manipulation/plan_to_pose -- pose frame
```


| Parameter | Type | Description |
| :--- | :--- | :--- |
| `pose` | `geometry_msgs/Pose` | 6D pose |
| `frame` | `string` | Frame of the pose |

### plan xyz in Cartesian space in the end-effector's frame

plan_cartesian_xyz = rospy.Service('/amiga/offline_manipulation/plan_cartesian_xyz', PlanCartesian, self.plan_cartesian_xyz)

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `x` | `float64` | changes in x |
| `y` | `float64` | changes in y |
| `z` | `float64` | changes in z |

example usage (go down by 20cm):
```
rosservice call /amiga//amiga/offline_manipulation/plan_cartesian_xyz -- 0.0 0.0 -0.2
```

### plan in joint configuration space

```
plan_joint_goal = rospy.Service('/amiga/offline_manipulation/plan_to_joint_goal', PlanJointGoal, self.plan_to_joint_goal)
plan_joints = rospy.Service('/amiga/offline_manipulation/plan_joints', PlanJointGoal, self.plan_joints) 
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `joint0` | `float64` | (changes in) radian angle |
| `joint1` | `float64` | (changes in) radian angle |
| `joint2` | `float64` | (changes in) radian angle |
| `joint3` | `float64` | (changes in) radian angle |
| `joint4` | `float64` | (changes in) radian angle |
| `joint5` | `float64` | (changes in) radian angle |

example usage (go to a joint configuration):
```
/amiga/offline_manipulation/plan_to_joint_goal -- joint0 joint1 joint2 joint3 joint4 joint5
```

example usage (change in current joint configuration):
```
/amiga/offline_manipulation/plan_joints -- djoint0 djoint1 djoint2 djoint3 djoint4 djoint5
```


### plan to traverse waypoints with the end-effector

```
plan_to_traverse = rospy.Service('/amiga/offline_manipulation/traverse_waypoints', TraverseWaypoints, self.plan_traversing_waypoints)
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `poses` | `geometry_msgs/Pose[]` | waypoints |

example usage (use perception to generate some waypoints):
```
rosservice call /amiga/offline_manipulation/traverse_waypoints poses
```

### utils

```
/amiga/offline_manipulation/get_current_eef_pose
```
Return the current eef pose

```
/amiga/offline_manipulation/publish_to_tf
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `pose` | `geometry_msgs/Pose pose` | 6D transform |
| `parent` | `string` | parent frame |
| `child` | `string` | child frame |
publish a tf transform (i.e. for visualisation)
 
 
## mid-level services 

### collisions

 ```
/amiga/offline_manipulation/add_to_collision_scene
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `name` | `string` | name of the obstacle |
| `cx` | `float64` | centre of x |
| `cy` | `float64` | centre of y |
| `cz` | `float64` | centre of z |
| `dx` | `float64` | length of x |
| `dy` | `float64` | length of y |
| `dz` | `float64` | length of z |

add a named cuboid shape obstacle to the collision scene

### virual-effector translator 

The virtual effector is assumed to be consisting of only a `translation` w.r.t eef 

vir pose &#8594; eef pose

vir trajectory &#8594; eef trajectory

 ```
/amiga/offline_manipulation/virtual_effector_pose_translator
```
| Parameter | Type | Description |
| :--- | :--- | :--- |
| `vir_pose` | `geometry_msgs/Pose` | a 6D pose |
| `translation_wrt_eef` | `geometry_msgs/Point` | translational transformation w.r.t. eef|

Return a geometry_msgs/Pose eef_pose

 ```
/amiga/offline_manipulation/virtual_effector_traj_translator
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `vir_poses` | `geometry_msgs/Pose[]` | a 6D pose |
| `translation_wrt_eef` | `geometry_msgs/Point` | translational transformation w.r.t. eef|

return geometry_msgs/Pose eef_poses[] 

## high-level services (made with low&mid levels and APIs optimisable)

### grasp executor

Given a pose in any frame, an intended gripper grasp type, and the desired approaching distance, plan a grasp accounting for gripper-arm offset.
Approach and then move to the target

![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_manip/media/grasp_executor.png)

 ```
/amiga/offline_manipulation/grasp_executor
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `target_pose` | `geometry_msgs/Pose[]` | a 6D grasp pose |
| `frame` | `geometry_msgs/Pose[]` | a 6D grasp pose |
| `distance` | `geometry_msgs/Point` | an approach distance |
| `gripper_type` | `geometry_msgs/Point` | gripper mode|

![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_manip/media/spatula.gif)


### circle executor 

Generate a circle with an adjustable radius and execute n times
 ```
/amiga/offline_manipulation/circle_executor
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `radius` | `float64` | radius of the circle |
| `number` | `int32` | number of executions |

For example, pair up with circle detection to perform a simple stirring

![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_manip/media/bowl.gif)

![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_manip/media/mix.gif)

### viewpoint adjuster

Generate a semi-sphere of viewpoints with adjustable sphere_layer, sphere_radius, longitude and latitude around a reference frame for the arm to reach so as to adjust viewpoints

![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_manip/media/view.png)

 ```
/amiga/offline_manipulation/view_point_adjuster
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `default` | `bool` | if use default param |
| `frame` | `string` | the reference frame |
| `param` | `float64` | spherical samling params |
| `publish_tf` | `bool` | if publish tfs|
| `execution` | `bool` | if execute|

![Alt Text](https://github.com/yw14218/amiga_extra/blob/main/amiga_manip/media/observe.gif)









