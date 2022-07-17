#!/usr/bin/env python3.6
import rospy
import os
from PIL import Image
from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse
import numpy as np
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
import tf2_ros
from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from vision_msgs.msg import Detection2DArray

class DexGraspGenerator():
    def __init__(self) -> None:
        super(DexGraspGenerator, self).__init__()
        rospy.Subscriber("obj_detect/detectnet/detections", Detection2DArray, self.callback_detnet)
        rospy.Subscriber("/zed2_node/point_cloud/cloud_registered", PointCloud2, self.callback_pc)

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

if __name__ == "__main__":

    rospy.init_node("gpd_grasp_rr", anonymous=True)
    
    pipeline = DexGraspGenerator()
    os.chdir('/home/yilong/catkin_ws/src/deep_grasp_demo/moveit_task_constructor_dexnet/data/images')
    rgb="/home/yilong/color_0.png"
    depth="/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images/zed/my_depth_image.png"
    # image1 = Image.open(rgb)
    # image2 = Image.open(depth)

    with open('/home/yilong/depth.npy', 'rb') as f:
        depth_cvmat = np.load(f)[...,None]

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

    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z


    ## transform

    r, p, y = euler_from_quaternion((rx, ry, rz, rw))
    dex_target = quaternion_matrix(quaternion_from_euler(r, p, y))
    eef_dex = quaternion_matrix(quaternion_from_euler(math.pi -math.pi/2, math.pi/4))


    eef_target = quaternion_from_matrix(np.dot(eef_dex, dex_target))

    listener = tf.TransformListener()
    listener.waitForTransform("zed2_left_camera_optical_frame", "amiga_arm_tool0", rospy.Time(), rospy.Duration(4.0))
    translation, rotation = listener.lookupTransform("zed2_left_camera_optical_frame", "amiga_arm_tool0", rospy.Time(0))

    q = [rotation[0], rotation[1], rotation[2], rotation[3]]
    cameraOptical_eef = quaternion_matrix(q)
    camera_target = quaternion_from_matrix(np.dot(cameraOptical_eef, eef_target))

    target_pose.orientation.x = camera_target[0]
    target_pose.orientation.y = camera_target[1]
    target_pose.orientation.z = camera_target[2]
    target_pose.orientation.w = camera_target[3]


    target_pose_bl = pipeline.transform_pose(target_pose, 'zed2_left_camera_optical_frame', 'base_link')     

    pose_in_list_bl = [target_pose_bl.position.x, target_pose_bl.position.y, target_pose_bl.position.z, 
            target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]
    pipeline.static_tf_broadcast('base_link', "grasp_baselink", pose_in_list_bl)
    
    # rospy.signal_shutdown('finished')
    rospy.spin()