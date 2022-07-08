import rospy
import tf2_ros
import numpy as np
import ros_numpy
import struct
import tf
import math
import time
from typing import Union
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, Image
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
from typing import List
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal

datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}

id_dict = {
    'bottle' : 44,
    'plate' : 26,# 45,
    'wine glass' : 46,
    'cup' : 47,
    'fork' : 48,
    'knife' : 49,
    'spoon' : 50,
    'bowl' : 51,
    'banana' : 52,
    'apple' : 53,
    'sandwich' : 54,
    'orange' : 55,
    'broccoli' : 56,
    'carrot' : 57,
    'hot dog' : 58,
    'pizza' : 59,
    'donut' : 60,
    'cake' : 61,
    'blender' : 83,
    'teddy bear' : 31 # 88 
} # interested classes in COCO dataset

class ObjDetGraspPipeline():
    def __init__(self) -> None:

        rospy.Subscriber("obj_detect/detectnet/detections", Detection2DArray, self.callback_detnet)
        rospy.Subscriber("/zed2_node/point_cloud/cloud_registered", PointCloud2, self.callback_pc)
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.callback_darknet)
        rospy.Subscriber("/zed2_node/depth/depth_registered", Image, self.callback_depth)
        #rospy.Subscriber("/zed2_node/rgb/image_rect_color", Image, self.callback_rgb)
        self.pub_pose = rospy.Publisher(
            '/amiga_grasp/obj_det/grasp_pose',
            Pose,
            queue_size=100
        )
        self.object = "teddy bear"
        self.grasp_link = "target link"
        self.grasp_pose = "top"
        self.plate = "plate"
        self.gripper_arm_offset = 0.15
        self.xCenterObject = None
        self.yCenterObject = None
        self.xplate = None
        self.yPlate = None
        self.pose1 = [1.1170107, -1.623156, 1.3613568, -1.5009832, -0.8726646, -2.932153]
        self.xyzplate = [0.85604518, 0.362330287694, 0.258105218410]
        self.detected_3dObject = [0.6330448985099, -0.36022070050, 0.183822736144]
        self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        self.gripper_init.call()
        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)

        self.inspect1_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_inspect1_pose", Empty)
        self.inspect2_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_inspect2_pose", Empty)

        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose_goal", PlanToPose)
        self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.gripper_wide_mode = rospy.ServiceProxy("/amiga_gripper/wide_mode_gripper", Empty)
        self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joint_goal_srv = rospy.ServiceProxy('/amiga/offline_manipulation/plan_joints', PlanJointGoal)

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



if __name__ == "__main__":

    rospy.init_node("test_grasp", anonymous=True)
    pose_in_list = [0.8270310957793867, -0.1192083898563673, -0.0771259397005252, 
    0.6629045488071652, 0.6688486679314914, -0.3342576849807776, -0.038351253021636195]

    # plate_3dObjectRobotFrame = pipeline.camera_to_robot(self.xyzplate, 'zed2_left_camera_frame', 'base_link')
    #                 target_pose = Pose(plate_3dObjectRobotFrame, Quaternion(0, 0, 0, 1))
    
    pipeline = ObjDetGraspPipeline()
