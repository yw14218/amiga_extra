from copy import deepcopy
from re import S

from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion, PoseStamped, Point
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal, TraverseWaypoints, GetPose, GraspExecutor, VirTransTraj, PublishTF
from math import pi, sin, cos
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
import rospy
import tf2_ros
import numpy as np


class Pipeline():

    def __init__(self) -> None:
        self.object = None

        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)
        self.go_to_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_home_pose", Empty)
        self.close_gripper_srv = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose", PlanToPose)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joints_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_joints", PlanJointGoal)
        self.traverse_waypoints = rospy.ServiceProxy("/amiga/offline_manipulation/traverse_waypoints", TraverseWaypoints)
        self.get_eef_pose = rospy.ServiceProxy("/amiga/offline_manipulation/get_current_eef_pose", GetPose)
        self.grasp_executor = rospy.ServiceProxy("/amiga/offline_manipulation/grasp_executor", GraspExecutor)
        self.trajectory_translate = rospy.ServiceProxy("/amiga/offline_manipulation/virtual_effector_traj_translator", VirTransTraj)
        self.pub_tf = rospy.ServiceProxy('/amiga/offline_manipulation/publish_to_tf', PublishTF)
    
    @staticmethod
    def list_to_pose(l):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = l[0], l[1], l[2], l[3], l[4], l[5], l[6]

        return pose

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
        
        print("Transformation sent")
    
    def pour(self):
        waypoints = []
        wpose = self.get_eef_pose.call()
        wpose = wpose.pose
        print(wpose)
        
        z = 0.4
        y = 0.3
        theta = -140
        theta *= pi / 180

        wpose.position.y = 0.05
        wpose.position.z = 0.3
        
        r, p, y = euler_from_quaternion((wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w))
        q = quaternion_from_euler(r, p, y)
        q1 = quaternion_from_euler(theta, 0, 0)

        r33 = quaternion_matrix(q)
        r33_1 = quaternion_matrix(q1)
        _r33 = np.dot(r33_1, r33)
        
        q = quaternion_from_matrix(_r33)

        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]
        
        waypoints.append(wpose)
        self.traverse_waypoints(waypoints)

        rospy.spin()
        


if __name__ == "__main__":

    rospy.init_node("simple_grasp_sim", anonymous=True)
    pipeline = Pipeline()
    pipeline.pour()
    
    rospy.spin()