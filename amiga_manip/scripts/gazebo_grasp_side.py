from copy import deepcopy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal, TraverseWaypoints, GetPose, GraspExecutor, AddCollision
import rospy
import time


class SimpleGraspPipeline():
    """
    In this type of grasp we get the intended object's pose from Gazebo directly, 
    there is no object detection Currently only support grasping from top
    """
    def __init__(self) -> None:
        super(SimpleGraspPipeline, self).__init__()
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
        self.add_collision = rospy.ServiceProxy("/amiga/offline_manipulation/add_to_collision_scene", AddCollision)
    
    def set_grasp_object(self, object : str):
        self.object = object

    def add_table(self, pose):
        self.add_collision.call("workbench", pose.position.x, pose.position.y, pose.position.z, 0.75, 1.0, 0.035)
    
    def grasp(self, object_pose : Pose):
        """
        Perform grasp operation
        """
        # go to grasp position
        target_pose = self.transform_pose_to_front_pick(object_pose)
        target_pose.position.z += 0.11
        target_pose.position.x -= 0.3
        
        #self.plan_to_pose_goal_srv.call(target_pose, "base_link")
        time.sleep(0.1)
         # go down and grasp
        # self.plan_cartesian_srv.call(0.0, 0.0, -0.2)
       
        self.grasp_executor.call(target_pose, "dummy_link", 0.15, "basic")
    
        time.sleep(0.1)
        self.close_gripper_srv.call()
        time.sleep(0.1)
        
    def manip(self):
        pose_list = []
        get_pose = self.get_eef_pose.call()
        pose_now = get_pose.pose
        pose_now.position.y += 0.4
        pose_list.append(deepcopy(pose_now))
        pose_now.position.y -= 0.8
        pose_list.append(deepcopy(pose_now))

        self.traverse_waypoints.call(pose_list)

    @staticmethod
    def transform_pose_to_front_pick(pose : Pose()) -> Pose:
        """
        setting an absolute orientation for grasp (from the side)
        """
        pose.orientation.x = 0.49956006551396753
        pose.orientation.y = 0.5004395699227152
        pose.orientation.z = 0.49991382218159247
        pose.orientation.w = 0.5000861407708072

        return pose

if __name__ == "__main__":

    rospy.init_node("simple_grasp_sim", anonymous=True)
    get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    pipeline = SimpleGraspPipeline()
    pipeline.set_grasp_object('coke_can')
    object_pose = get_pose_srv.call(pipeline.object, "robot").pose
    pipeline.add_table(object_pose)

    pipeline.grasp(object_pose)

    # go up
    #pipeline.plan_cartesian_srv.call(0.0, 0.0, 0.6)
    
    #pipeline.go_to_home_srv.call()
    # manip
    #pipeline.manip()

    rospy.spin()
