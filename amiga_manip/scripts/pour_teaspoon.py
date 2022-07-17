from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal, TraverseWaypoints, GetPose, GraspExecutor, PublishTF
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
        self.close_gripper_srv = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose", PlanToPose)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joints_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_joints", PlanJointGoal)
        self.traverse_waypoints = rospy.ServiceProxy("/amiga/offline_manipulation/traverse_waypoints", TraverseWaypoints)
        self.get_eef_pose = rospy.ServiceProxy("/amiga/offline_manipulation/get_current_eef_pose", GetPose)
        self.grasp_executor = rospy.ServiceProxy("/amiga/offline_manipulation/grasp_executor", GraspExecutor)
        self.pub_tf = rospy.ServiceProxy('/amiga/offline_manipulation/publish_to_tf', PublishTF)
        
    def pour(self):
        waypoints = []
        wpose = self.get_eef_pose.call()
        wpose = wpose.pose
        print(wpose)
        
        theta = 60

        theta *= pi / 180

        for dtheta in np.arange(0, theta, theta/100):
            if dtheta == 0:
                pass
            print(dtheta)
            r, p, y = euler_from_quaternion((wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w))
            q = quaternion_from_euler(r, p, y)
        
            q1 = quaternion_from_euler(dtheta, 0, 0)

            r33 = quaternion_matrix(q)
            r33_1 = quaternion_matrix(q1)

            _r33 = np.dot(r33_1, r33)

            q = quaternion_from_matrix(_r33)
            wpose.position.y -= 0.12 * sin(dtheta)
            wpose.position.z -= 0.12 * (1 - cos(dtheta))
            wpose.orientation.x = q[0]
            wpose.orientation.y = q[1]
            wpose.orientation.z = q[2]
            wpose.orientation.w = q[3]
            waypoints.append(deepcopy(wpose))

        # self.go_to_grasp_home_srv.call()
        # self.plan_cartesian_srv.call(0.0, 0.0, -0.15)

        # theta = 60 * pi / 180
        # r, p, y = euler_from_quaternion((wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w))
        # q = quaternion_from_euler(r, p, y)

        # q1 = quaternion_from_euler(theta, 0, 0)

        # r33 = quaternion_matrix(q)
        # r33_1 = quaternion_matrix(q1)

        # _r33 = np.dot(r33_1, r33)

        # q = quaternion_from_matrix(_r33)
        # wpose.position.y -= 0.2 * sin(theta)
        # wpose.position.z -= 0.2 * (1 - cos(theta))
        # wpose.orientation.x = q[0]
        # wpose.orientation.y = q[1]
        # wpose.orientation.z = q[2]
        # wpose.orientation.w = q[3]
        # waypoints.append(deepcopy(wpose))

        # self.traverse_waypoints(waypoints)

        # time.sleep(0.5)
        # self.plan_cartesian_srv.call(0.0, 0.0, -0.15)
        # time.sleep(0.5)
        # self.plan_joints_srv.call(0.0, 0.0, 0.0, 0.0, 0.5, 0.0)
        #self.go_to_grasp_home_srv.call()

        
        

if __name__ == "__main__":

    rospy.init_node("simple_grasp_sim", anonymous=True)
    pipeline = Pipeline()
    pipeline.pour()
    
    rospy.spin()
