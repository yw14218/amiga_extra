#!/usr/bin/env python3.8
import moveit_commander
import moveit_msgs.msg
import rospy
import sys
import tf
import numpy as np
import tf2_ros
import struct
import os
from copy import deepcopy
from geometry_msgs.msg import Pose, Vector3, TransformStamped
from typing import Union, List, Tuple, Optional
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import math

# os.environ['ROS_IP'] = '10.0.3.15'
# os.environ['ROS_MASTER_URI']="http://10.0.3.11:11311"

class AmigaMovegroup(object):
    """
    Wrapper of move_group for amiga
    """
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander("arm_no_gripper")
        self.hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
        self.arm_group.set_pose_reference_frame("base_link") # important
        #self.arm_group.set_planning_frame("base_link")
        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_goal_orientation_tolerance(0.01)
        self.arm_group.allow_replanning(False)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

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

    def set_grasping_home_pose(self):
        self.set_group_pose('arm', 'grasping_home_arm')

    def set_grasping_top_pose(self):
        self.set_group_pose('arm', 'grasping_top_arm')

    def set_gripper_close_pose(self):
        """
        sim only
        """
        self.set_group_pose('hand', 'gripper_closed')

    def set_gripper_open_pose(self):
        """
        sim only
        """
        self.set_group_pose('hand', 'gripper_open')

    def go_up(self, height : float = 0.6):
        waypoints = []
        wpose = self.arm_group.get_current_pose().pose
        wpose.position.z += height / 2
        waypoints.append(deepcopy(wpose))
        wpose.position.z += height / 2 
        waypoints.append(deepcopy(wpose))
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        self.arm_group.execute(plan, wait=True)
        self.arm_group.clear_pose_targets()
        
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
    def transform_pose_to_top_pick(pose : Pose()) -> Pose:
        """
        setting an absolute orientation for grasp (from the top)
        """
        pose.orientation.x = -0.8903691114115917
        pose.orientation.y = -0.45523902110162956
        pose.orientation.z = 0.00015021799829083296
        pose.orientation.w = 0.0005065028287560422

        return pose

    @staticmethod
    def transform_pose_to_side_pick(pose : Pose()) -> Pose:
        """
        setting an absolute orientation for grasp (from the side)
        """
        pose.orientation.x = 0.49956006551396753
        pose.orientation.y = 0.5004395699227152
        pose.orientation.z = 0.49991382218159247
        pose.orientation.w = 0.5000861407708072

        return pose

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
        print("sent")

    @staticmethod
    def get_xyz(point_2d, pc_msg):
        datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}
        arrayPosition = point_2d[0]*pc_msg.row_step + point_2d[1]*pc_msg.point_step # point_2d: y,x
        pos_x = arrayPosition + pc_msg.fields[0].offset # X has an offset of 0
        len_x = datatype[pc_msg.fields[0].datatype]
        pos_y = arrayPosition + pc_msg.fields[1].offset # Y has an offset of 4
        len_y = datatype[pc_msg.fields[1].datatype]
        pos_z = arrayPosition + pc_msg.fields[2].offset # Z has an offset of 8
        len_z = datatype[pc_msg.fields[2].datatype]

        try:
            x = struct.unpack('f', pc_msg.data[pos_x: pos_x+len_x])[0] # read 4 bytes as a float number
            y = struct.unpack('f', pc_msg.data[pos_y: pos_y+len_y])[0]
            z = struct.unpack('f', pc_msg.data[pos_z: pos_z+len_z])[0]
            return [x,y,z]
        except:
            return None

    # @staticmethod
    # def palm_to_endeffector(pose_3d : Union[list, Vector3], palm_link : str, endeffector_link : str) -> Vector3:
    #     """
    #     transform a 3d point from the camera frame to the robot frame
    #     """
    #     if isinstance(pose_3d, Vector3):
    #         pose_3d = [pose_3d.x, pose_3d.y, pose_3d.z]

    #     listener = tf.TransformListener()
    #     listener.waitForTransform(robot_link, camera_link, rospy.Time(), rospy.Duration(4.0))
    #     translation, rotation = listener.lookupTransform(robot_link, camera_link, rospy.Time(0))
    #     cam2rob = np.dot(tf.transformations.translation_matrix(translation), tf.transformations.quaternion_matrix(rotation)) # build a 3x4 3D affine matrix
    #     pose_3d_rob = np.dot(cam2rob, np.array(pose_3d+[1]))

    #     return Vector3(pose_3d_rob[0], pose_3d_rob[1], pose_3d_rob[2])

if __name__ == "__main__":
    """
    In simulation: amiga_arm_tool0 link has no non-zero translation w.r.t amiga_palm_link, but does have relative rotationot
    """
    pipeline = AmigaMovegroup()
    rospy.init_node("amiga_movegroup", anonymous=True)


    n = 1
    dq = -math.pi/2 
    dq_dn = dq / n
    x, y, z = 0, 0, 0
    for i in range (n):
        waypoints = []
        wpose = pipeline.arm_group.get_current_pose().pose
        print(wpose)
        x, y, z = wpose.position.x, wpose.position.y, wpose.position.z
        r, p, y = euler_from_quaternion((wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w))
        q = quaternion_from_euler(r, p, y)
        q1 = quaternion_from_euler(-math.pi/4, 0, 0)

        r33 = quaternion_matrix(q)
        r33_1 = quaternion_matrix(q1)

        _r33 = np.dot(r33_1, r33)

        q = quaternion_from_matrix(_r33)

        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]
        waypoints.append(deepcopy(wpose))

        (plan, fraction) = pipeline.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        plan.joint_trajectory.header.frame_id="base_link"
        pipeline.arm_group.execute(plan, wait=True)
        pipeline.arm_group.clear_pose_targets()

        waypoints = []
        wpose = pipeline.arm_group.get_current_pose().pose
        print(wpose) # xyz of eef unchanged

    
    