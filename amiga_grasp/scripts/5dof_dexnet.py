#!/usr/bin/env python3.6
import rospy
import os
from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
from turtle import pos
import rospy
import os
import tf2_ros
import numpy as np
import open3d as o3ds
import math
from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from sensor_msgs.msg import PointCloud2, Image
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, PoseStamped
from moveit_task_constructor_gpd.srv import PointCloud
import numpy as np
import tf2_ros
import os
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion
from typing import Union, List, Tuple, Optional
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, rotation_matrix
from numpy.linalg import inv
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal
from vision_msgs.msg import Detection2DArray
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal, TraverseWaypoints, GetPose, GraspExecutor

class DexGraspPipeline():
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
        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)
        self.go_to_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_home_pose", Empty)
        self.close_gripper_srv = rospy.ServiceProxy("/amiga/offline_manipulation/simulation/close_gripper", Empty)
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose", PlanToPose)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joints_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_joints", PlanJointGoal)
        self.traverse_waypoints = rospy.ServiceProxy("/amiga/offline_manipulation/traverse_waypoints", TraverseWaypoints)
        self.get_eef_pose = rospy.ServiceProxy("/amiga/offline_manipulation/get_current_eef_pose", GetPose)
        self.grasp_executor = rospy.ServiceProxy("/amiga/offline_manipulation/grasp_executor", GraspExecutor)


    def callback_detnet(self, data):
        for detection in data.detections:
            if detection is not None and detection.results[0].id == 18: # fry pan
                self.xCenterObject = int(detection.bbox.center.x)
                self.yCenterObject = int(detection.bbox.center.y)

    
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

rospy.spin()