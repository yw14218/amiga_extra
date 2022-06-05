#!/usr/bin/env python3.6

from pyrsistent import pset_field
import rospy
import os
from PIL import Image
from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse
from amiga_group import AmigaMovegroup
import numpy as np

rospy.init_node("gpd_grasp_rr", anonymous=True)
os.chdir('/home/yilong/catkin_ws/src/deep_grasp_demo/moveit_task_constructor_dexnet/data/images')
rgb="/home/yilong/color_0.png"
depth="/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images/zed/my_depth_image.png"
image1 = Image.open(rgb)
image2 = Image.open(depth)
print(image1.format)
print(image1.size)
print(image1.mode)
print(image2.format)
print(image2.size)
print(image2.mode)
with open('/home/yilong/depth.npy', 'rb') as f:
    depth_cvmat = np.load(f)[...,None]

print("~~~~~~~")
print(depth_cvmat.shape)
print(depth_cvmat.dtype)
# print(depth_cvmat)
rospy.wait_for_service("/gqcnn_grasp", 10.0)
proxy = rospy.ServiceProxy("/gqcnn_grasp", GQCNNGrasp)
result = proxy.call(rgb, depth)
x = result.grasps[0].pose.position.x
y = result.grasps[0].pose.position.y
z = result.grasps[0].pose.position.z
rx = result.grasps[0].pose.orientation.x
ry = result.grasps[0].pose.orientation.y
rz = result.grasps[0].pose.orientation.z
rw = result.grasps[0].pose.orientation.w
pose_in_list = [x, y, z, rx ,ry, rz, rw]
print(pose_in_list)
AmigaMovegroup.static_tf_broadcast("zed2_left_camera_frame", "dummy_link", pose_in_list)

rospy.spin()