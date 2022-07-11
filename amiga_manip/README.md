# amiga_manip

A hierarchical manipulation server for Amiga based on MoveIt. 

## Core concept

* Complex manipulation tasks can be described as a combination of simpler ones. The purpose of this server 
is to identify and provide several useful services to Amiga developers such that they can concentrate on 
investigating how to use perception to generate a desired online trajectory and action plan.

* A single manipulation server is more organised, extensible and trackable than spamming scripts for each task.

* Services can be used in a script via proxies.

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

### plan xyz in cartesian space in the end-effector's frame
plan_cartesian_xyz = rospy.Service('/amiga/offline_manipulation/plan_cartesian_xyz', PlanCartesian, self.plan_cartesian_xyz)
| Parameter | Type | Description |
| :--- | :--- | :--- |
| `x` | `float64` | changes in x |
| `y` | `float64` | changes in y |
| `z` | `float64` | changes in z |

example usage (lift by 20cm):
```
rosservice call /amiga//amiga/offline_manipulation/plan_cartesian_xyz -- 0.0 0.0 0.2
```

# plan to traverse waypoints
plan_to_traverse = rospy.Service('/amiga/offline_manipulation/traverse_waypoints', TraverseWaypoints, self.plan_traversing_waypoints)




# plan in joint configuration space
plan_joint_goal = rospy.Service('/amiga/offline_manipulation/plan_to_joint_goal', PlanJointGoal, self.plan_to_joint_goal)
plan_joints = rospy.Service('/amiga/offline_manipulation/plan_joints', PlanJointGoal, self.plan_joints) 
 
 
 
 
view_point_adjuster = rospy.Service('/amiga/offline_manipulation/view_point_adjuster', ViewAdjust, self.adjust_viewpoints)
circle_executor = rospy.Service('/amiga/offline_manipulation/circle_executor', CircleExecutor, self._circle_executor)
add_to_collision_scene = rospy.Service('/amiga/offline_manipulation/add_to_collision_scene', AddCollision, self.add_to_collision_scene)
virtual_effector_pose_translator = rospy.Service('/amiga/offline_manipulation/virtual_effector_pose_translator', 
    VirTransPose, self._virtual_effect_translate_pose)
virtual_effector_traj_translator = rospy.Service('/amiga/offline_manipulation/virtual_effector_traj_translator', 
    VirTransTraj, self._virtual_effect_translate_traj)
grasp_executor = rospy.Service('/amiga/offline_manipulation/grasp_executor', 
    GraspExecutor, self._grasp_executor)



# utils
get_current_eef_pose = rospy.Service('/amiga/offline_manipulation/get_current_eef_pose', GetPose, self.get_current_pose)
publish_to_tf = rospy.Service('/amiga/offline_manipulation/publish_to_tf', PublishTF, self.publish_to_tf)
        
To authenticate an API request, you should provide your API key in the `Authorization` header.

Alternatively, you may append the `api_key=[API_KEY]` as a GET parameter to authorize yourself to the API. But note that this is likely to leave traces in things like your history, if accessing the API through a browser.

```http
GET /api/campaigns/?api_key=12345678901234567890123456789012
```
