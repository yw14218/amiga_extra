#!/usr/bin/env python3.6

from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse
import os
import rospy


rospy.init_node("test_dex", anonymous=True)


os.chdir('/home/yilong/catkin_ws/src/deep_grasp_demo/moveit_task_constructor_dexnet/data/images')

rgb="/home/yilong/color_0.png"
depth="/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images/zed/my_depth_image.png"

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


rospy.spin()