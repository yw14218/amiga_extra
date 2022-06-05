#!/usr/bin/env python3.8
import moveit_commander
import moveit_msgs.msg
import rospy
import sys
import tf
import numpy as np
import tf2_ros
from copy import deepcopy
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion
from typing import Union, List, Tuple, Optional
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty, EmptyResponse
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanCartesianResponse, PlanToPoseResponse, PlanJointGoal, PlanJointGoalResponse

class AmigaMovegroupServer(object):
    """
    Wrapper of move_group for amiga
    """
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander("arm_no_gripper")
        self.hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
        #self.arm_group.set_planning_frame("base_link") # right now not supported by MoveIt yet
    
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
        plan_to_grasp_home_pose = rospy.Service('/amiga/offline_manipulation/go_to_grasp_home_pose', Empty, self.set_grasping_home_pose)
        plan_to_front_pose = rospy.Service('/amiga/offline_manipulation/go_grasp_front_pose', Empty, self.set_grasping_front_pose)
        plan_to_zero_pose = rospy.Service('/amiga/offline_manipulation/go_to_zero_pose', Empty, self.set_zero_pose)
        plan_to_home_pose = rospy.Service('/amiga/offline_manipulation/go_to_home_pose', Empty, self.set_home_pose)

        plan_to_inspect1_pose = rospy.Service('/amiga/offline_manipulation/go_to_inspect1_pose', Empty, self.set_inspect1_pose)
        plan_to_inspect2_pose = rospy.Service('/amiga/offline_manipulation/go_to_inspect2_pose', Empty, self.set_inspect2_pose)

        plan_to_pose_goal = rospy.Service('/amiga/offline_manipulation/plan_to_pose_goal', PlanToPose, self.plan_to_pose_goal)
        plan_to_pose_goal_coord_transform = rospy.Service('/amiga/offline_manipulation/plan_to_pose_goal_coord_transform', PlanToPose, self.plan_to_pose_goal_coord_transform)
        plan_cartesian_xyz = rospy.Service('/amiga/offline_manipulation/plan_cartesian_xyz', PlanCartesian, self.plan_cartesian_xyz)
        
        plan_joint_goal = rospy.Service('/amiga/offline_manipulation/plan_to_joint_goal', PlanJointGoal, self.plan_to_joint_goal)
        plan_joints = rospy.Service('/amiga/offline_manipulation/plan_joints', PlanJointGoal, self.plan_joints)

        # sim only
        if simulation == True:
            close_gripper = rospy.Service('/amiga/offline_manipulation/simulation/close_gripper', Empty, self.set_gripper_close_pose)
            open_gripper = rospy.Service('/amiga/offline_manipulation/simulation/open_gripper', Empty, self.set_gripper_open_pose)

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
        
    def plan_cartesian_path(self, point_3d_rob : Union[List, Tuple] = None, quaternion : Union[List, Tuple] = None, waypoints : Optional[List] = []):
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
        self.arm_group.execute(plan, wait=True)
        self.arm_group.clear_pose_targets()
        
        return plan, fraction
    
    def plan_pose_goal(self, point_3d : Union[List, Tuple] = None, quaternion : Union[List, Tuple] = None, pose : Optional[Pose] = None):
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

    def plan_to_pose_goal(self, req):
        pose_goal = Pose()
        pose_goal.position.x = req.x
        pose_goal.position.y = req.y
        pose_goal.position.z = req.z
        pose_goal.orientation.x = req.rx
        pose_goal.orientation.y = req.ry
        pose_goal.orientation.z = req.rz
        pose_goal.orientation.w = req.rw
        pose_in_list = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, 
            pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        self.static_tf_broadcast('base_link', 'pose_target_link', pose_in_list)
        plan = self.plan_pose_goal(pose = pose_goal)
        #self.display_trajectory(plan)

        if plan is not None:
            return PlanToPoseResponse(True)
        else:
            return PlanToPoseResponse(False)

    def plan_to_pose_goal_coord_transform(self, req):
        pose_3d = [req.x, req.y, req.z]
        detected_3dObjectRobotFrame = self.camera_to_robot(pose_3d, 'zed2_left_camera_frame', 'base_link')
        target_pose = Pose(detected_3dObjectRobotFrame, Quaternion(req.rx, req.ry, req.rz, req.rw))

        pose_in_list = [target_pose.position.x, target_pose.position.y, target_pose.position.z, 
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
        self.static_tf_broadcast('base_link', 'pose_target_link', pose_in_list)
        plan = self.plan_pose_goal(pose = target_pose)
        #self.display_trajectory(plan)

        if plan is not None:
            return PlanToPoseResponse(True)
        else:
            return PlanToPoseResponse(False)

    def plan_cartesian_xyz(self, req):
        
        waypoints = []
        wpose = self.arm_group.get_current_pose().pose
        #print(wpose)
        wpose.position.x += req.x
        wpose.position.y += req.y
        wpose.position.z += req.z
        waypoints.append(deepcopy(wpose))
        #print(wpose)
        self.arm_group.clear_pose_targets()
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        #print(plan)
        self.arm_group.execute(plan, wait=True)
        self.arm_group.clear_pose_targets()

        if plan is not None:
            return PlanCartesianResponse(True)
        else:
            return PlanCartesianResponse(False)

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

    @staticmethod
    def camera_to_robot(pose_3d : Union[list, Vector3], camera_link : str, robot_link : str ='base_link') -> Vector3:
        """
        transform a 3d point from the camera frame to the robot frame
        """
        if isinstance(pose_3d, Vector3):
            pose_3d = [pose_3d.x, pose_3d.y, pose_3d.z]

        listener = tf.TransformListener()
        listener.waitForTransform(robot_link, camera_link, rospy.Time(), rospy.Duration(4.0))
        translation, rotation = listener.lookupTransform(robot_link, camera_link, rospy.Time(0))
        cam2rob = np.dot(tf.transformations.translation_matrix(translation), tf.transformations.quaternion_matrix(rotation)) # build a 3x4 3D affine matrix
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
        

if __name__ == "__main__":
    rospy.init_node("amiga_manipulation_server", anonymous=True)
    manipulation_server = AmigaMovegroupServer()

    rospy.spin()
