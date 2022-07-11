import rospy
import open3d as o3d
import tf2_ros
import tf
import numpy as np
import open3d as o3ds
import math
from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from sensor_msgs.msg import PointCloud2, Image
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, PoseStamped
from moveit_task_constructor_gpd.srv import PointCloud
import rospy
import numpy as np
import tf2_ros
from copy import deepcopy
from geometry_msgs.msg import Pose, Vector3, TransformStamped
from typing import Union, List, Tuple, Optional
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
from numpy.linalg import inv

class VirtualGraspffector():
    def __init__(self) -> None:

        rospy.init_node("virtual_effector", anonymous=True)

    @staticmethod
    def static_tf_broadcast(parent_id : str, child_id : str, pose_in_list : List) -> None:
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
        print("sent")

    def start(self):

        # q = quaternion_from_euler(-5*math.pi/4, -math.pi/2, -3*math.pi/2)
        # pose_frame = [0.4, 0.4, 0.25, q[0], q[1], q[2], q[3]]
        # self.static_tf_broadcast('amiga_arm_tool0', "gpd_grasp_frame", pose_frame)

        # pose_in_list = [0, 0, 0.2, 0, 0, 0, 1]
        # self.static_tf_broadcast("amiga_arm_tool0", "virtual_effector", pose_in_list)
        
        # pose_in_list_vs = [-0.07, 0.07, 0.26, 0, 0, 0, 1]
        # self.static_tf_broadcast("amiga_arm_tool0", "virtual_scissor", pose_in_list_vs)


        # q = quaternion_from_euler(math.pi, -math.pi/2, math.pi/4)
        # pose_frame = [0.4, 0.4, 0.25, q[0], q[1], q[2], q[3]]
        # self.static_tf_broadcast('amiga_arm_tool0', "dex_net_frame", pose_frame)
        pass


if __name__ == "__main__":

    virtual_effector = VirtualGraspffector()
    virtual_effector.start()

    rospy.spin()
