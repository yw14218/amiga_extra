# amiga_manip

A hierarchical manipulation server for Amiga based on MoveIt. 

## Core concept

Complex manipulation tasks can be described as a combination of simpler ones. The purpose of this server is to identify and provide some useful services to Amiga developers such that they can concentrate on using perception to generate a desired online trajectory and action plan.

 A single manipulation server is more organized, extensible and trackable than spamming scripts for each task.


All API requests require the use of a generated API key. You can find your API key, or generate a new one, by navigating to the /settings endpoint, or clicking the “Settings” sidebar item.

view_point_adjuster = rospy.Service('/amiga/offline_manipulation/view_point_adjuster', ViewAdjust, self.adjust_viewpoints)
circle_executor = rospy.Service('/amiga/offline_manipulation/circle_executor', CircleExecutor, self._circle_executor)
add_to_collision_scene = rospy.Service('/amiga/offline_manipulation/add_to_collision_scene', AddCollision, self.add_to_collision_scene)
virtual_effector_pose_translator = rospy.Service('/amiga/offline_manipulation/virtual_effector_pose_translator', 
    VirTransPose, self._virtual_effect_translate_pose)
virtual_effector_traj_translator = rospy.Service('/amiga/offline_manipulation/virtual_effector_traj_translator', 
    VirTransTraj, self._virtual_effect_translate_traj)
grasp_executor = rospy.Service('/amiga/offline_manipulation/grasp_executor', 
    GraspExecutor, self._grasp_executor)

# lower-level

# go to pre-defined poses
plan_to_grasp_home_pose = rospy.Service('/amiga/offline_manipulation/go_to_grasp_home_pose', Empty, self.set_grasping_home_pose)
plan_to_front_pose = rospy.Service('/amiga/offline_manipulation/go_grasp_front_pose', Empty, self.set_grasping_front_pose)
plan_to_zero_pose = rospy.Service('/amiga/offline_manipulation/go_to_zero_pose', Empty, self.set_zero_pose)
plan_to_home_pose = rospy.Service('/amiga/offline_manipulation/go_to_home_pose', Empty, self.set_home_pose)
plan_to_inspect1_pose = rospy.Service('/amiga/offline_manipulation/go_to_inspect1_pose', Empty, self.set_inspect1_pose)
plan_to_inspect2_pose = rospy.Service('/amiga/offline_manipulation/go_to_inspect2_pose', Empty, self.set_inspect2_pose)

# plan to a given pose in a frame
plan_to_pose_goal = rospy.Service('/amiga/offline_manipulation/plan_to_pose', PlanToPose, self.plan_to_pose_goal)

# plan to traverse waypoints
plan_to_traverse = rospy.Service('/amiga/offline_manipulation/traverse_waypoints', TraverseWaypoints, self.plan_traversing_waypoints)

# plan in cartesian space xyz
plan_cartesian_xyz = rospy.Service('/amiga/offline_manipulation/plan_cartesian_xyz', PlanCartesian, self.plan_cartesian_xyz)


# plan in joint configuration space
plan_joint_goal = rospy.Service('/amiga/offline_manipulation/plan_to_joint_goal', PlanJointGoal, self.plan_to_joint_goal)
plan_joints = rospy.Service('/amiga/offline_manipulation/plan_joints', PlanJointGoal, self.plan_joints)

# utils
get_current_eef_pose = rospy.Service('/amiga/offline_manipulation/get_current_eef_pose', GetPose, self.get_current_pose)
publish_to_tf = rospy.Service('/amiga/offline_manipulation/publish_to_tf', PublishTF, self.publish_to_tf)
        
To authenticate an API request, you should provide your API key in the `Authorization` header.

Alternatively, you may append the `api_key=[API_KEY]` as a GET parameter to authorize yourself to the API. But note that this is likely to leave traces in things like your history, if accessing the API through a browser.

```http
GET /api/campaigns/?api_key=12345678901234567890123456789012
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `api_key` | `string` | **Required**. Your Gophish API key |
