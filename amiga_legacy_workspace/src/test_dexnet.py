#!/usr/bin/env python3.6
from pyrsistent import pset_field
from sklearn import pipeline
import rospy
import os
from PIL import Image
from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse
from amiga_group import AmigaMovegroup
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
from turtle import pos
import rospy
import actionlib
import os
import yaml
import time
import open3d as o3d
import tf2_ros
import numpy as np
import open3d as o3ds
import math
from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from sensor_msgs.msg import PointCloud2, Image
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, PoseStamped
from moveit_task_constructor_gpd.srv import PointCloud
import moveit_commander
import moveit_msgs.msg
import rospy
import sys
import tf
import numpy as np
import tf2_ros
import struct
import os
import moveit_msgs.msg
from copy import deepcopy
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion
from typing import Union, List, Tuple, Optional
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, rotation_matrix
from amiga_group import AmigaMovegroup
from numpy.linalg import inv
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal
from vision_msgs.msg import Detection2DArray

class DexGraspPipeline(AmigaMovegroup):
    def __init__(self) -> None:
        super(DexGraspPipeline, self).__init__()
        # rospy.wait_for_service("/save_point_cloud", 10.0)
        # rospy.wait_for_service("/amiga_gripper/init_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/open_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/close_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/pinch_mode_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/basic_mode_gripper", 10.0)
        rospy.Subscriber("obj_detect/detectnet/detections", Detection2DArray, self.callback_detnet)
        rospy.Subscriber("/zed2_node/point_cloud/cloud_registered", PointCloud2, self.callback_pc)
        self.collect_pc = rospy.ServiceProxy("/save_point_cloud", PointCloud)
        self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.gripper_set_pinch_mode = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.gripper_set_basic_mode = rospy.ServiceProxy("/amiga_gripper/basic_gripper", Empty)
        self.gripper_set_scissor_mode = rospy.ServiceProxy("/amiga_gripper/scissor_gripper", Empty)
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose_goal", PlanToPose)
        self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.gripper_wide_mode = rospy.ServiceProxy("/amiga_gripper/wide_mode_gripper", Empty)
        self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joint_goal_srv = rospy.ServiceProxy('/amiga/offline_manipulation/plan_joints', PlanJointGoal)
        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)

        self.xCenterObject = None
        self.yCenterObject = None
        self.detected_3dObject = None


    def callback_detnet(self, data):
        for detection in data.detections:
            if detection is not None and detection.results[0].id == 18: # fry pan
                self.xCenterObject = int(detection.bbox.center.x)
                self.yCenterObject = int(detection.bbox.center.y)
                #print("center_X: " + str(self.xCenterObject) + " center_Y: " + str(self.yCenterObject))

    
    def callback_pc(self, data):
        # m = ros_numpy.numpify(data) shape : (720, 1280)
        if self.xCenterObject is not None and self.yCenterObject is not None: 
            detected_3dObject = self.get_xyz([self.yCenterObject, self.xCenterObject], data)
            self.detected_3dObject = detected_3dObject
            print(self.detected_3dObject)

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
    def static_tf_broadcast(parent_id : str, child_id : str, pose_in_list) -> None:
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
        print("tf of target link successfully sent")



rospy.init_node("gpd_grasp_rr", anonymous=True)

p = PoseStamped()
pipeline = DexGraspPipeline()
p.header.frame_id = pipeline.robot.get_planning_frame()
p.pose.position.x = 0.82
p.pose.position.y = 0.16
p.pose.position.z = -0.15

p_bar = PoseStamped()
p_bar.header.frame_id = pipeline.robot.get_planning_frame()
p_bar.pose.position.x = 1.82
p_bar.pose.position.y = 0.73
p_bar.pose.position.z = 0.5


for i in range(100):
    pipeline.scene.add_box("workbench", p, (1.5, 1.5, 0.035))
    pipeline.scene.add_box("bar", p, (1.0, 0.035, 1.0))
    


os.chdir('/home/yilong/catkin_ws/src/deep_grasp_demo/moveit_task_constructor_dexnet/data/images')
rgb="/home/yilong/color_0.png"
depth="/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images/zed/my_depth_image.png"
# image1 = Image.open(rgb)
# image2 = Image.open(depth)

with open('/home/yilong/depth.npy', 'rb') as f:
    depth_cvmat = np.load(f)[...,None]


# print(depth_cvmat)
rospy.wait_for_service("/gqcnn_grasp", 10.0)
proxy = rospy.ServiceProxy("/gqcnn_grasp", GQCNNGrasp)
result = proxy.call(rgb, depth)
x = result.grasps[0].pose.position.x
y = result.grasps[0].pose.position.y
z = result.grasps[0].pose.position.z - 0.11
rx = result.grasps[0].pose.orientation.x
ry = result.grasps[0].pose.orientation.y
rz = result.grasps[0].pose.orientation.z
rw = result.grasps[0].pose.orientation.w
pose_in_list = [x, y, z, rx ,ry, rz, rw]
print(pose_in_list)
AmigaMovegroup.static_tf_broadcast("zed2_left_camera_optical_frame", "dex_link", pose_in_list)

target_pose = Pose()
target_pose.position.x = x
target_pose.position.y = y
target_pose.position.z = z - 0.11




## transform

r, p, y = euler_from_quaternion((rx, ry, rz, rw))

print("rpy:",r, p, y)

waypoints = []
wpose = pipeline.arm_group.get_current_pose().pose
qx = wpose.orientation.x
qy = wpose.orientation.y
qz = wpose.orientation.z
qw = wpose.orientation.w
cr, cp, cy = euler_from_quaternion((qx,qy,qz,qw))
print(cr, cp, cy)
_q = quaternion_from_euler(cr, cp, cy + p)

# dex = quaternion_matrix(quaternion_from_euler(r,p,y))
# r33 = quaternion_matrix(quaternion_from_euler(-math.pi, math.pi/2, -math.pi/4))
# _q = quaternion_from_matrix(np.dot(dex, r33))


target_pose.orientation.x = _q[0]
target_pose.orientation.y = _q[1]
target_pose.orientation.z = _q[2]
target_pose.orientation.w = _q[3]

# pose_in_list = pose_in_list[:3] + list(quaternion_from_euler(*pose_in_list[3:]))

target_pose_bl = AmigaMovegroup.transform_pose(target_pose, 'zed2_left_camera_optical_frame', 'base_link')

# r_bl, p_bl, y_bl = euler_from_quaternion((target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w))
# r33 = quaternion_matrix(quaternion_from_euler(r_bl, p_bl, y_bl))
# r33 = np.delete(r33, 3, 0)
# r33 = np.delete(r33, 3, 1)
# dxdydz = np.dot(r33, np.array([0, 0, 0.10]))
# x, y, z = [target_pose_bl.position.x, target_pose_bl.position.y, target_pose_bl.position.z]
# _x = x - dxdydz[0]
# _y = y - dxdydz[1]
# _z = z - dxdydz[2]       

# pose_in_list_bl = [target_pose_bl.position.x, target_pose_bl.position.y, target_pose_bl.position.z, 
#         target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]
# AmigaMovegroup.static_tf_broadcast('base_link', "grip_baselink", pose_in_list_bl)
# pose_in_list_eef = [_x, _y, _z, 
#     target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]
# AmigaMovegroup.static_tf_broadcast('base_link', "eef_baselink", pose_in_list_eef)

# waypoints = []
# wpose = pipeline.arm_group.get_current_pose().pose
# wpose.position.x = _x
# wpose.position.y = _y
# wpose.position.z = _z
# wpose.orientation.x = target_pose_bl.orientation.x
# wpose.orientation.y = target_pose_bl.orientation.y
# wpose.orientation.z = target_pose_bl.orientation.z
# wpose.orientation.w = target_pose_bl.orientation.w
# waypoints.append(deepcopy(wpose))

detected_3dObjectRobotFrame = None
time.sleep(3)
if pipeline.detected_3dObject is not None:
    pipeline.detected_3dObject = [0.6787927746772766, 0.35561561584472656, 0.13231943547725677]
    detected_3dObjectRobotFrame = pipeline.camera_to_robot(pipeline.detected_3dObject, 'zed2_left_camera_frame', 'base_link')


else:
    pipeline.detected_3dObject = [0.6787927746772766, 0.35561561584472656, 0.13231943547725677]
    detected_3dObjectRobotFrame = pipeline.camera_to_robot(pipeline.detected_3dObject, 'zed2_left_camera_frame', 'base_link')

# waypoints = []
# wpose = pipeline.arm_group.get_current_pose().pose
# wpose.position.x = detected_3dObjectRobotFrame.x - 0.1
# wpose.position.y = detected_3dObjectRobotFrame.y
# wpose.position.z = detected_3dObjectRobotFrame.z + 0.35
# waypoints.append(deepcopy(wpose))

# pose_in_list = [wpose.position.x, wpose.position.y, wpose.position.z, wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
# pipeline.static_tf_broadcast('base_link', 'teddy_bear_grasp_link', pose_in_list)


# (plan, fraction) = pipeline.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
# plan.joint_trajectory.header.frame_id="base_link"

# pipeline.arm_group.execute(plan, wait=True)
# pipeline.arm_group.clear_pose_targets()


# wpose.position.y = detected_3dObjectRobotFrame.y
# wpose.position.z = detected_3dObjectRobotFrame.z + 0.35
# waypoints.append(deepcopy(wpose))





# joint_goal = pipeline.arm_group.get_current_joint_values()
# print("spamming", joint_goal[4])
# joint_goal[4] += 1.57
# print("spamming", joint_goal[4])
# if joint_goal[4] < -math.pi:
#     joint_goal[4] += math.pi
# # elif joint_goal[5] > math.pi:
# #     joint_goal[5] - math.pi
# #pipeline.arm_group.go(joint_goal, wait=True)
# pipeline.arm_group.stop()


# joint_goal = pipeline.arm_group.get_current_joint_values()
# print("spamming", joint_goal[5])
# joint_goal[5] -= r
# print("spamming", joint_goal[5])
# if joint_goal[5] < -math.pi:
#      joint_goal[5] += math.pi
# # elif joint_goal[5] > math.pi:
# #     joint_goal[5] - math.pi
# pipeline.arm_group.go(joint_goal, wait=True)
# pipeline.arm_group.stop()

waypoints = []
wpose = pipeline.arm_group.get_current_pose().pose

r, p, y = euler_from_quaternion((wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w))
q = quaternion_from_euler(r, p, y)

pour = 0
q1 = quaternion_from_euler(-math.pi/2, 0, 0)
q2 = quaternion_from_euler(0, pour, 0)


r33 = quaternion_matrix(q)
r33_1 = quaternion_matrix(q1)
r33_2 = quaternion_matrix(q2)

_r33 = np.dot(np.dot(r33_2, r33_1), r33)

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


# waypoints = []
# wpose = pipeline.arm_group.get_current_pose().pose
# wpose.position.x = target_pose_bl.position.x
# wpose.position.y = target_pose_bl.position.y
# wpose.position.z = target_pose_bl.position.z
# waypoints.append(deepcopy(wpose))




# # waypoints = []
# # wpose = pipeline.arm_group.get_current_pose().pose
# # wpose.orientation.x = target_pose_bl.orientation.x
# # wpose.orientation.y = target_pose_bl.orientation.y
# # wpose.orientation.z = target_pose_bl.orientation.z
# # wpose.orientation.w = target_pose_bl.orientation.w
# # waypoints.append(deepcopy(wpose))




# file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan_dex.yaml')
# with open(file_path, 'w') as file_save:
#     yaml.dump(plan, file_save, default_flow_style=True)

# with open(file_path, 'r') as file_open:
#     loaded_plan = yaml.load(file_open, Loader=yaml.Loader)

# pipeline.arm_group.execute(plan, wait=True)
# pipeline.arm_group.clear_pose_targets()

# time.sleep(3)
# pipeline.gripper_close.call()
# time.sleep(3)
# pipeline.go_to_grasp_home_srv.call()

# rospy.signal_shutdown('finished')
rospy.spin()