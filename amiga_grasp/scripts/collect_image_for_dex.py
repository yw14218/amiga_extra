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

class DexPipeline():
    def __init__(self) -> None:

        # rospy.Subscriber("obj_detect/detectnet/detections", Detection2DArray, self.callback_detnet)
        #rospy.Subscriber("/zed2_node/point_cloud/cloud_registered", PointCloud2, self.callback_pc)
        # rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.callback_darknet)
        rospy.Subscriber("/zed2_node/depth/depth_registered", Image, self.callback_depth)
        rospy.Subscriber("/zed2_node/rgb/image_rect_color", Image, self.callback_rgb)
        os.chdir("/home/yilong/git_ws/src/ur10e_robotiq/amiga_grasp/data/tmp")

    def callback_rgb(self, data): #(540, 940, 4) 1080 ''' (621, 1104, 4) 2k 
        data = ros_numpy.numpify(data)
        data = data[0:270, 480:960]
        print(data.shape)
        print(data.dtype)
        with open('rgb.npy', 'wb') as f:
            np.save(f, data)

    def callback_depth(self, data): #(540, 940) ''' (621, 1104, 4) 2k 
        data = ros_numpy.numpify(data)
        data = data[0:270, 480:960]
        print(data.shape)
        print(data.dtype)
        with open('depth.npy', 'wb') as f:

            np.save(f, data)

if __name__ == "__main__":
    rospy.init_node("grasp_dex", anonymous=True)
    pipeline = DexPipeline()

    rospy.spin()
