import rospy
import tf2_ros
import numpy as np
import ros_numpy
import struct
import tf
import math
import os
import time
from typing import Union
import yaml
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, Image
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
from typing import List
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal
import cv2
import cv_bridge
from cv_bridge import CvBridgeError
import os
datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}

class DexPipeline():
    def __init__(self) -> None:

        # rospy.Subscriber("obj_detect/detectnet/detections", Detection2DArray, self.callback_detnet)
        #rospy.Subscriber("/zed2_node/point_cloud/cloud_registered", PointCloud2, self.callback_pc)
        # rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.callback_darknet)
        rospy.Subscriber("/zed2_node/depth/depth_registered", Image, self.callback_depth)
        rospy.Subscriber("/zed2_node/rgb/image_rect_color", Image, self.callback_rgb)
        self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        self.gripper_init.call()
        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose_goal", PlanToPose)
        self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        os.chdir("/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/tmp")

    def callback_rgb(self, data): #(540, 940, 4) 1080 ''' (621, 1104, 4) 2k 
        data = ros_numpy.numpify(data)
        data = data[0:270, 470:940]
        print(data.shape)
        print(data.dtype)
        with open('rgb.npy', 'wb') as f:

            np.save(f, data)



    def callback_depth(self, data): #(540, 940) ''' (621, 1104, 4) 2k 
        data = ros_numpy.numpify(data)
        data = data[0:270, 0:470]
        print(data.shape)
        print(data.dtype)
        with open('depth.npy', 'wb') as f:

            np.save(f, data)
        # try:
        #     depth_array = ros_numpy.numpify(data)
        #     print(depth_array.dtype)
        #     print(depth_array.shape)
        #     cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        #     print(data.encoding)
        #     cv2.imwrite("depth.tif", depth_array*255)

        #     file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'depth.yaml')
        #     with open(file_path, 'w') as file_save:
        #         yaml.dump(depth_array, file_save, default_flow_style=True)
        # except CvBridgeError as e:
        #     print(e)

if __name__ == "__main__":
    rospy.init_node("grasp_dex", anonymous=True)
    pipeline = DexPipeline()

    rospy.spin()
