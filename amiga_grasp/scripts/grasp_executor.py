from copy import deepcopy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal
import rospy
import time
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
from tf.transformations import euler_from_quaternion
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_multiply
from amiga_group import AmigaMovegroup
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped


class GraspExecutor(AmigaMovegroup):

    def __init__(self) -> None:

        super(GraspExecutor, self).__init__()
        # self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)
        # self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose_goal", PlanToPose)
        # self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        # self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        # self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        # self.gripper_init()

    def grasp(self, target_pose : Pose):
        """
        Perform grasp operation
        """
        res = self.plan_to_pose_goal_srv.call(target_pose.position.x, target_pose.position.y, target_pose.position.z,
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
        assert res is True
        time.sleep(0.1)

        self.gripper_close.call()
        time.sleep(0.1)
        

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

    # @staticmethod
    # def _grasp_planner(pose_3d : Union[list, Vector3], grasp_link : str ='grip_baselink', robot_link : str ='base_link') -> Vector3:
    #     """
    #     transform a 3d point from the camera frame to the robot frame
    #     """
    #     if isinstance(pose_3d, Vector3):
    #         pose_3d = [pose_3d.x, pose_3d.y, pose_3d.z]

    #     listener = tf.TransformListener()
    #     listener.waitForTransform(robot_link, grasp_link, rospy.Time(), rospy.Duration(4.0))
    #     translation, rotation = listener.lookupTransform(robot_link, grasp_link, rospy.Time(0))

    #     listener.waitForTransform("amiga_arm_tool0", "virtual_effector", rospy.Time(), rospy.Duration(4.0))



    #     grasp2rob = np.dot(tf.transformations.translation_matrix(translation), tf.transformations.quaternion_matrix(rotation)) # build a 3x4 3D affine matrix
    #     pose_3d_rob = np.dot(cam2rob, np.array(pose_3d+[1]))

    #     return Vector3(pose_3d_rob[0], pose_3d_rob[1], pose_3d_rob[2])

if __name__ == "__main__":

    rospy.init_node("grasp_executor", anonymous=True)

    # rospy.wait_for_service("/amiga/offline_manipulation/go_to_grasp_home_pose", 10.0)
    # rospy.wait_for_service("/amiga/offline_manipulation/plan_to_pose_goal", 10.0)
    # rospy.wait_for_service("/amiga_gripper/close_gripper", 10.0)
    
    pipeline = GraspExecutor()
    p = PoseStamped()
    p.header.frame_id = pipeline.robot.get_planning_frame()
    p.pose.position.x = 0.82
    p.pose.position.y = 0.15
    p.pose.position.z = -0.15
    pipeline.scene.add_box("workbench", p, (1.0, 1.0, 0.035))

    
    # pipeline.arm_group.set_end_effector_link("virtual_effector")
    # eef_link = pipeline.arm_group.get_end_effector_link()
    # print("============ End effector link: %s" % eef_link)

    # listener = tf.TransformListener()
    # listener.waitForTransform('grip_baselink', 'base_link', rospy.Time(), rospy.Duration(4.0))
    # translation, rotation = listener.lookupTransform('base_link', 'grip_baselink', rospy.Time(0))
  
    # pipeline.plan_to_pose_goal_srv.call(
    #     translation[0], translation[1], translation[2] + 0.15,
    #     rotation[0], rotation[1], rotation[2], rotation[3]
    # )

    # pipeline.arm_group.set_pose_target(pose_goal)
    # plan = pipeline.arm_group.go(wait=True)
    # pipeline.arm_group.stop()
    # pipeline.arm_group.clear_pose_targets()

    # waypoints = []
    # wpose = pipeline.arm_group.get_current_pose().pose
    # waypoints.append(deepcopy(wpose))
    # wpose.position.x = translation[0]
    # wpose.position.y = translation[1]
    # wpose.position.z = translation[2] + 0.22
    # wpose.orientation.x = rotation[0]
    # wpose.orientation.y = rotation[1]
    # wpose.orientation.z = rotation[2]
    # wpose.orientation.w = rotation[3]
    # (plan, fraction) = pipeline.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    # plan.joint_trajectory.header.frame_id="base_link"
    # print(plan)
    # pipeline.arm_group.execute(plan, wait=True)
    # pipeline.arm_group.clear_pose_targets()

    # waypoints = []
    # wpose = pipeline.arm_group.get_current_pose().pose
    # wpose.position.x = 0.5458639435487418
    # wpose.position.y = 0.32577818571358064
    # wpose.position.z = 0.10070187689783289
    # wpose.orientation.x = 0.34970560807392864
    # wpose.orientation.y = 0.8117665189815129
    # wpose.orientation.z = 0.14827272370446834
    # wpose.orientation.w = 0.4435722102967448
    # waypoints.append(deepcopy(wpose))
    # (plan, fraction) = pipeline.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    # plan.joint_trajectory.header.frame_id="base_link"
    # print(plan)
    # pipeline.arm_group.execute(plan, wait=True)
    # pipeline.arm_group.clear_pose_targets()

    # time.sleep(3)
    # pipeline.gripper_close.call()
    # time.sleep(3)
    # pipeline.go_to_grasp_home_srv.call()
    # rospy.signal_shutdown("end")
    

    # # grasp
    # pipeline.grasp(target_pose)

    # # go up
    # pipeline.plan_cartesian_srv.call(0.0, 0.0, 0.5)

    rospy.spin()

    


