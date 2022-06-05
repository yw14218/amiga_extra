from copy import deepcopy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal
import rospy
import time


class SimpleGraspPipeline():
    """
    In this type of grasp we get the intended object's pose from Gazebo directly, there is no object detection
    Currently only support grasping from top
    """
    def __init__(self) -> None:
        super(SimpleGraspPipeline, self).__init__()
        self.object = None

        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)
        self.close_gripper_srv = rospy.ServiceProxy("/amiga/offline_manipulation/simulation/close_gripper", Empty)
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose_goal", PlanToPose)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joints_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_joints", PlanJointGoal)

    def set_grasp_object(self, object : str):
        self.object = object

    def grasp(self, object_pose : Pose):
        """
        Perform grasp operation
        """
        # go to grasp position
        target_pose = self.transform_pose_to_top_pick(object_pose)
        target_pose.position.z += 0.7
        
        self.plan_to_pose_goal_srv.call(target_pose.position.x, target_pose.position.y, target_pose.position.z,
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
        time.sleep(0.1)
         # go down and grasp
        self.plan_cartesian_srv.call(0.0, 0.0, -0.4)
        time.sleep(0.1)
        self.close_gripper_srv.call()
        time.sleep(0.1)

    @staticmethod
    def transform_pose_to_top_pick(pose : Pose()) -> Pose:
        """
        setting an absolute orientation for grasp (from the top)
        """
        pose.orientation.x = -0.8903691114115917
        pose.orientation.y = -0.45523902110162956
        pose.orientation.z = 0.00015021799829083296
        pose.orientation.w = 0.0005065028287560422

        return pose

if __name__ == "__main__":

    rospy.init_node("simple_grasp_sim", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", 10.0)
    rospy.wait_for_service("/amiga/offline_manipulation/go_to_grasp_home_pose", 10.0)
    rospy.wait_for_service("/amiga/offline_manipulation/plan_to_pose_goal", 10.0)
    rospy.wait_for_service("/amiga/offline_manipulation/simulation/close_gripper", 10.0)  
    get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    
    pipeline = SimpleGraspPipeline()
    pipeline.set_grasp_object('beer')
    object_pose = get_pose_srv.call(pipeline.object, "robot").pose

    # go home
    pipeline.go_to_grasp_home_srv.call()
    pipeline.grasp(object_pose)

    # go up
    pipeline.plan_cartesian_srv.call(0.0, 0.0, 0.8)

    # manip
    pipeline.plan_joints_srv.call(0.0, 0.0, 0.0, 0.0, 0.0, -0.5)
    rospy.spin()
