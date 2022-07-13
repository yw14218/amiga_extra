#!/usr/bin/env python3.8
import moveit_commander
import moveit_msgs.msg
import rospy
import sys
import tf
import numpy as np
import tf2_ros
from copy import deepcopy
from math import pi, cos , sin
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion, PoseStamped
from typing import Union, List, Tuple, Optional
from tf.transformations import quaternion_from_euler, quaternion_matrix
from std_srvs.srv import Empty, EmptyResponse
import tf2_geometry_msgs
from amiga_manip.srv import (PlanCartesian, PlanToPose, PlanCartesianResponse, 
    PlanToPoseResponse, PlanJointGoal, PlanJointGoalResponse, PublishTF, PublishTFResponse,
    GetPose, GetPoseResponse, TraverseWaypoints, TraverseWaypointsResponse,
    ViewAdjust, ViewAdjustResponse, AddCollision, AddCollisionResponse,
    VirTransPose, VirTransPoseResponse, VirTransTraj, VirTransTrajResponse,
    GraspExecutor, GraspExecutorResponse, CircleExecutor, CircleExecutorResponse)

class AmigaManipulationServer(object):
    """
    Amiga manipulation server
        "Everthing is a service"
        TODO:
            Separate planning and executing stage -- plan & visualise then execute
            Introduce more parameters such wapoints traversing resolution
    """
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander("arm_no_gripper")
        self.hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
        # self.arm_group.set_planning_frame("base_link") # right now not supported by MoveIt yet
    
        goal_position_tolerance = rospy.get_param('~goal_position_tolerance') # 0.005
        goal_orientation_tolerance = rospy.get_param('~goal_orientation_tolerance') # 0.005
        allow_replanning = rospy.get_param('~allow_replanning') # false
        pose_reference_frame = rospy.get_param('~pose_reference_frame') # base_link
        simulation = rospy.get_param('~simulation')

        self.arm_group.set_goal_position_tolerance(goal_position_tolerance)
        self.arm_group.set_goal_orientation_tolerance(goal_orientation_tolerance)
        self.arm_group.allow_replanning(allow_replanning)
        self.arm_group.set_pose_reference_frame(pose_reference_frame) # important as default is "dummy_link".

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # services
        
        # higher-level
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

        # sim only
        if simulation == True:
            close_gripper = rospy.Service('/amiga/offline_manipulation/simulation/close_gripper', Empty, self.set_gripper_close_pose)
            open_gripper = rospy.Service('/amiga/offline_manipulation/simulation/open_gripper', Empty, self.set_gripper_open_pose)

    @staticmethod
    def pose_to_list(pose):
        return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def set_group_pose(self, group : moveit_commander.move_group.MoveGroupCommander, pose : Pose):
        if group == 'arm':
            self.arm_group.set_named_target(pose)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
        elif group == 'hand':
            self.hand_group.set_named_target(pose)
            self.hand_group.go(wait=True)
            self.hand_group.stop()
            self.hand_group.clear_pose_targets()
        else:
            raise ValueError('no such group')

    def set_grasping_home_pose(self, req):
        self.set_group_pose('arm', 'grasping_home_arm')
        return EmptyResponse()

    def set_grasping_front_pose(self, req):
        self.set_group_pose('arm', 'grasping_front_arm')
        return EmptyResponse()
        
    def set_zero_pose(self, req):
        self.set_group_pose('arm', 'zero')
        return EmptyResponse()

    def set_inspect1_pose(self, req):
        self.set_group_pose('arm', 'grasp_insepect1')
        return EmptyResponse()

    def set_inspect2_pose(self, req):
        self.set_group_pose('arm', 'grasp_insepect2')
        return EmptyResponse()
    
    def set_home_pose(self, req):
        self.set_group_pose('arm', 'home')
        return EmptyResponse()

    def set_gripper_close_pose(self, req):
        """
        sim only
        """
        self.set_group_pose('hand', 'gripper_closed')
        return EmptyResponse()

    def set_gripper_open_pose(self, req):
        """
        sim only
        """
        self.set_group_pose('hand', 'gripper_open')
        return EmptyResponse()
        
    def plan_cartesian_path(self, point_3d_rob : Union[List, Tuple] = None, 
            quaternion : Union[List, Tuple] = None, waypoints : Optional[List] = []):

        if point_3d_rob is not None and quaternion is not None:
            pose_goal = Pose()
            pose_goal.orientation.x = quaternion[0]
            pose_goal.orientation.y = quaternion[1]
            pose_goal.orientation.z = quaternion[2]
            pose_goal.orientation.w = quaternion[3]
            # TODO
            pose_goal.position.x = point_3d_rob[0]
            pose_goal.position.y = point_3d_rob[1]
            pose_goal.position.z = point_3d_rob[2]

            waypoints.append(pose_goal)

        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)  
        self.display_trajectory(plan)
        # cartesian plans default to be in odom frame for Amiga
        plan.joint_trajectory.header.frame_id="base_link"
        self.arm_group.execute(plan, wait=True)
        self.arm_group.clear_pose_targets()
        
        return plan, fraction
    
    def plan_pose_goal(self, point_3d : Union[List, Tuple] = None, 
            quaternion : Union[List, Tuple] = None, pose : Optional[Pose] = None):
        if point_3d is not None and quaternion is not None:
            pose_goal = Pose()
            pose_goal.orientation.x = quaternion[0]
            pose_goal.orientation.y = quaternion[1]
            pose_goal.orientation.z = quaternion[2]
            pose_goal.orientation.w = quaternion[3]
            pose_goal.position.x = point_3d[0]
            pose_goal.position.y = point_3d[1]
            pose_goal.position.z = point_3d[2]

        elif pose is not None:
            pose_goal = pose
        
        else:
            raise TypeError
            
        self.arm_group.set_pose_target(pose_goal)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        return plan

    def publish_to_tf(self, req):
        pose_goal = req.pose
        pose_in_list = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, 
            pose_goal.orientation.x, pose_goal.orientation.y, 
                pose_goal.orientation.z, pose_goal.orientation.w]
        self.static_tf_broadcast(req.parent, req.child, pose_in_list)

        return PublishTFResponse()

    def plan_to_pose_goal(self, req):
        if req.frame != 'base_link':
            pose_bl = self.transform_pose(req.pose, req.frame, 'base_link')
        else:
            pose_bl = req.pose
        plan = self.plan_pose_goal(pose = pose_bl)
        #self.display_trajectory(plan)

        if plan is not None:
            return PlanToPoseResponse(True)
        else:
            return PlanToPoseResponse(False)

    def plan_cartesian_xyz(self, req):
        
        waypoints = []
        wpose = self.arm_group.get_current_pose().pose
        wpose.position.x += req.x
        wpose.position.y += req.y
        wpose.position.z += req.z
        waypoints.append(deepcopy(wpose))
        self.arm_group.clear_pose_targets()
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        plan.joint_trajectory.header.frame_id="base_link"
        self.arm_group.execute(plan, wait=True)
        self.arm_group.clear_pose_targets()

        if plan is not None:
            return PlanCartesianResponse(True)
        else:
            return PlanCartesianResponse(False)

    def plan_traversing_waypoints(self, req):
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    req.poses,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         #
        plan.joint_trajectory.header.frame_id="base_link"
        self.arm_group.execute(plan, wait=True)
        self.arm_group.clear_pose_targets()
        
        if plan is not None:
            return TraverseWaypointsResponse(True)
        else:
            return TraverseWaypointsResponse(False)
            

    def plan_to_joint_goal(self, req):
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] = req.joint0
        joint_goal[1] = req.joint1
        joint_goal[2] = req.joint2
        joint_goal[3] = req.joint3
        joint_goal[4] = req.joint4
        joint_goal[5] = req.joint5
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.arm_group.go(joint_goal, wait=True)
        self.arm_group.stop()

        return PlanJointGoalResponse()

    def plan_joints(self, req):
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] += req.joint0
        joint_goal[1] += req.joint1
        joint_goal[2] += req.joint2
        joint_goal[3] += req.joint3
        joint_goal[4] += req.joint4
        joint_goal[5] += req.joint5
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.arm_group.go(joint_goal, wait=True)
        self.arm_group.stop()

        return PlanJointGoalResponse()

    def get_current_pose(self, req):
        poseCur = self.arm_group.get_current_pose().pose
        return GetPoseResponse(poseCur)

    @staticmethod
    def camera_to_robot(pose_3d : Union[list, Vector3], 
            camera_link : str, robot_link : str ='base_link') -> Vector3:
        """
        transform a 3d point from the camera frame to the robot frame
        """
        if isinstance(pose_3d, Vector3):
            pose_3d = [pose_3d.x, pose_3d.y, pose_3d.z]

        listener = tf.TransformListener()
        listener.waitForTransform(robot_link, camera_link, rospy.Time(), rospy.Duration(4.0))
        translation, rotation = listener.lookupTransform(robot_link, camera_link, rospy.Time(0))
        cam2rob = np.dot(tf.transformations.translation_matrix(translation), 
            tf.transformations.quaternion_matrix(rotation)) # build a 3x4 3D affine matrix
        pose_3d_rob = np.dot(cam2rob, np.array(pose_3d+[1]))

        return Vector3(pose_3d_rob[0], pose_3d_rob[1], pose_3d_rob[2])

    @staticmethod
    def static_tf_broadcast(parent_id : str, child_id : str, pose_in_list : List) -> None:
        br = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent_id
        static_transformStamped.child_frame_id = child_id
        static_transformStamped.transform.translation.x = pose_in_list[0]
        static_transformStamped.transform.translation.y = pose_in_list[1]
        static_transformStamped.transform.translation.z = pose_in_list[2]
        static_transformStamped.transform.rotation.x = pose_in_list[3]
        static_transformStamped.transform.rotation.y = pose_in_list[4]
        static_transformStamped.transform.rotation.z = pose_in_list[5]
        static_transformStamped.transform.rotation.w = pose_in_list[6]
        br.sendTransform(static_transformStamped)
        
        print("Transformation sent")
        
    @staticmethod
    def transform_pose(input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(2))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            raise e

    @staticmethod
    def spherical_sampling(frame='zed2_left_camera_optical_frame', 
            sphere_layer=2, sphere_radius = 0.4, longitude=3, latitude=4):
        """
        Generate a sphere of view points for the arm to reach for data collection purposes
        """
        tf_set = []
        pose_set = []

        # Centre              
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = frame
        static_transformStamped.child_frame_id = "point_centre"
        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0
        quat = quaternion_from_euler(-0.15785, 0.63107, 0.6549)
        static_transformStamped.transform.rotation.x = -quat[0]
        static_transformStamped.transform.rotation.y = -quat[1]
        static_transformStamped.transform.rotation.z = -quat[2]
        static_transformStamped.transform.rotation.w = -quat[3]
        tf_set.append(static_transformStamped)
        
        # Sampling spherical coordinates
        for l in range(sphere_layer):
            for i in range(longitude):
                for j in range(latitude):
                    theta = pi * (90 / (latitude + 1)) * ((latitude - 1) / 2 - j) / 180
                    phi = pi * (360 / longitude / 2) * i / 180
                    x = sphere_layer * sphere_radius * sin(theta) * cos(phi)
                    y = sphere_layer * sphere_radius * sin(theta) * sin(phi)
                    z = sphere_layer * sphere_radius * cos(theta) - 1.2
                    roll = theta * sin(-phi)
                    pitch = theta * cos(phi)
                    yaw = 0
                    frame_label = "point " + str(l) + "," + str(i) + "," + str(j)
                    quat = quaternion_from_euler(roll, pitch, yaw)
                    static_transformStamped = TransformStamped()
                    static_transformStamped.header.stamp = rospy.Time.now()
                    static_transformStamped.header.frame_id = frame
                    static_transformStamped.child_frame_id = frame_label
                    static_transformStamped.transform.translation.x = -(x + 0.0525)
                    static_transformStamped.transform.translation.y = -(y + 0.1057)
                    static_transformStamped.transform.translation.z = -(z + 0.128)
                    static_transformStamped.transform.rotation.x = -quat[0]
                    static_transformStamped.transform.rotation.y = -quat[1]
                    static_transformStamped.transform.rotation.z = -quat[2]
                    static_transformStamped.transform.rotation.w = -quat[3]
                    pose_set.append(Pose(Vector3(-(x + 0.0525), -(y + 0.1057), -(z + 0.128)), 
                        Quaternion(-quat[0], -quat[1], -quat[2], -quat[3])))
                    tf_set.append(static_transformStamped)

        return tf_set, pose_set

    def adjust_viewpoints(self, req):
        if req.default == True:
            tf_set, pose_set = self.spherical_sampling(frame='zed2_left_camera_optical_frame', 
                sphere_layer=2, sphere_radius = 0.4, longitude=3, latitude=4)
        else:
            tf_set, pose_set = self.spherical_sampling(frame=req.frame, 
                sphere_layer=req.param[0], sphere_radius = req.param[1], longitude=req.param[2], latitude=req.param[3])

        if req.publish_tf == True:
            br = tf2_ros.StaticTransformBroadcaster()
            br.sendTransform(tf_set)
        
        if req.execution == True:
            pose_bl_list = []
            for pose in pose_set:
                pose_bl = self.transform_pose(input_pose=pose, from_frame=req.frame, to_frame="base_link")
                pose_bl_list.append(pose_bl)
            for pose_bl in pose_bl_list:
                self.plan_pose_goal(pose=pose_bl)
        
        return ViewAdjustResponse()

    def add_to_collision_scene(self, req):
        """
            lab's floor from grasp_home_pose is:
            p.pose.position.x = 0.82
            p.pose.position.y = 0.15
            p.pose.position.z = -0.15
            self.scene.add_box("workbench", p, (1.0, 1.0, 0.035))
        """
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = req.cx
        p.pose.position.y = req.cy
        p.pose.position.z = req.cz
        self.scene.add_box(req.name, p, (req.dx,req.dy, req.dz))

        return AddCollisionResponse()

    @staticmethod
    def vitual_effect_translate_pose(pose:Pose, translation:Vector3):
        """
            translation is with respect to Amiga_arm_tool0
            for example, the gripper-arm offset is [0.0, 0.0, 0.20]
        """
        vir_pose = pose
        tr= translation
        q = Quaternion()
        q.x = vir_pose.orientation.x
        q.y = vir_pose.orientation.y
        q.z = vir_pose.orientation.z
        q.w = vir_pose.orientation.w
        r33 = quaternion_matrix([q.x, q.y, q.z, q.w])
        r33 = np.delete(r33, 3, 0)
        r33 = np.delete(r33, 3, 1)
        dxdydz = np.dot(r33, np.array([tr[0], tr[1], tr[2]]))
        x, y, z = vir_pose.position.x, vir_pose.position.y, vir_pose.position.z
        _x = x - dxdydz[0]
        _y = y - dxdydz[1]
        _z = z - dxdydz[2]       

        eef_pose = Pose()
        eef_pose.position.x = _x
        eef_pose.position.y = _y
        eef_pose.position.z = _z
        eef_pose.orientation.x = vir_pose.orientation.x
        eef_pose.orientation.y = vir_pose.orientation.y
        eef_pose.orientation.z = vir_pose.orientation.z
        eef_pose.orientation.w = vir_pose.orientation.w
        
        return eef_pose 

    def _virtual_effect_translate_pose(self, req):
        eef_pose = self.vitual_effect_translate_pose(req.vir_pose, req.translation_wrt_eef)

        return VirTransPoseResponse(eef_pose)

    def _virtual_effect_translate_traj(self, req):
        eef_traj = []
        for pose in req.vir_poses:
            eef_pose = self.vitual_effect_translate_pose(pose, req.translation_wrt_eef)
            eef_traj.append(eef_traj)

        return VirTransTrajResponse(eef_traj)

    def grasp_executor(self, target_pose, frame, distance, gripper_type):
        """
            Given a pose in any frame, an intended
            grripper grasp type, and a desired 
            approaching distance, plan a grasp
            accounting for gripper-arm offset.
            Approach and then move to target
        """

        if gripper_type == "basic":
            eef_pose_baselink = self.transform_pose(target_pose, from_frame=frame, to_frame='base_link')

            eef_pose_baselink = self.vitual_effect_translate_pose(target_pose, [0.0, 0.0, 0.20])
            self.static_tf_broadcast("base_link", "grasp_pose", self.pose_to_list(eef_pose_baselink))
            
            approach_pose_baselink = self.vitual_effect_translate_pose(target_pose, [0.0, 0.0, 0.20 + distance])
            self.static_tf_broadcast("base_link", "approach_pose", self.pose_to_list(approach_pose_baselink))

            waypoints = []
            waypoints.append(approach_pose_baselink)
            waypoints.append(eef_pose_baselink)
            (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)  

            self.plan_pose_goal(pose = approach_pose_baselink)
            self.plan_pose_goal(pose = eef_pose_baselink)
            
        
    def _grasp_executor(self, req):
        self.grasp_executor(req.target_pose, req.frame, req.distance, req.gripper_type)

        return GraspExecutorResponse()

    def _circle_executor(self, req):
        pose_now = self.arm_group.get_current_pose().pose
        cx = pose_now.position.x
        cy = pose_now.position.y
        wpoints = []
        for theta in np.arange(0, 2*pi, 0.01):
            x = cx + req.radius*cos(theta)
            y = cy + req.radius*sin(theta)
            pose_now.position.x = x
            pose_now.position.y = y
            wpoints.append(deepcopy(pose_now))

        for i in range (req.number):
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                                        wpoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         #
            plan.joint_trajectory.header.frame_id="base_link"
        
            self.arm_group.execute(plan, wait=True)
            self.arm_group.clear_pose_targets()
            
        return CircleExecutorResponse()

if __name__ == "__main__":
    rospy.init_node("amiga_manipulation_server", anonymous=True)
    manipulation_server = AmigaManipulationServer()

    rospy.spin()