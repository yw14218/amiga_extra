from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal, TraverseWaypoints, GetPose, GraspExecutor
from math import pi, sin, cos
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
import rospy
import time
import numpy as np


class Pipeline():

    def __init__(self) -> None:
        self.object = None

        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)
        self.go_to_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_home_pose", Empty)
        self.close_gripper_srv = rospy.ServiceProxy("/amiga/offline_manipulation/simulation/close_gripper", Empty)
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose", PlanToPose)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joints_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_joints", PlanJointGoal)
        self.traverse_waypoints = rospy.ServiceProxy("/amiga/offline_manipulation/traverse_waypoints", TraverseWaypoints)
        self.get_eef_pose = rospy.ServiceProxy("/amiga/offline_manipulation/get_current_eef_pose", GetPose)
        self.grasp_executor = rospy.ServiceProxy("/amiga/offline_manipulation/grasp_executor", GraspExecutor)
        
    def pour(self):

        self.go_to_grasp_home_srv.call()
        waypoints = []
        wpose = self.get_eef_pose.call()
        wpose = wpose.pose
        
        n = 10
        dtheta = 0.1
        for i in range(n):
            self.plan_joints_srv.call(0.0, 0.0, 0.0, 0.0, dtheta, 0.0)
            time.sleep(0.2)
            self.plan_cartesian_srv.call(-0.0005, -0.25 * sin(dtheta), 0.0)
            time.sleep(0.2)
            self.plan_cartesian_srv.call(0.0, 0.0, -4*(1 - cos(dtheta)))

        self.plan_joints_srv.call(0.0, 0.0, 0.0, 0.0, 0.5, 0.0)

if __name__ == "__main__":

    rospy.init_node("simple_grasp_sim", anonymous=True)
    pipeline = Pipeline()

    pipeline.pour()
    
    rospy.spin()
