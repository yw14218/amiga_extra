#!/usr/bin/env python3.6
import rospy
import os
from PIL import Image
from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
from turtle import pos
import rospy
import os
import open3d as o3d
import tf2_ros
import numpy as np
import open3d as o3ds
import math
from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
import rospy
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
        depth_cvmat = np.load(f)[..., None]

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

    target_pose.orientation.x = _q[0]
    target_pose.orientation.y = _q[1]
    target_pose.orientation.z = _q[2]
    target_pose.orientation.w = _q[3]

    # pose_in_list = pose_in_list[:3] + list(quaternion_from_euler(*pose_in_list[3:]))

    target_pose_bl = pipeline.transform_pose(target_pose, 'zed2_left_camera_optical_frame', 'base_link')


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


    joint_goal = pipeline.arm_group.get_current_joint_values()
    print("spamming", joint_goal[5])
    joint_goal[5] -= r
    print("spamming", joint_goal[5])
    if joint_goal[5] < -math.pi:
        joint_goal[5] += math.pi
    # elif joint_goal[5] > math.pi:
    #     joint_goal[5] - math.pi
    pipeline.arm_group.go(joint_goal, wait=True)
    pipeline.arm_group.stop()

    # rospy.signal_shutdown('finished')
    rospy.spin()
