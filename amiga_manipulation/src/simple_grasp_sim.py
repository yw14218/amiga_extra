from copy import deepcopy
from amiga_group import AmigaMovegroup
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

import rospy

class SimpleGraspPipeline(AmigaMovegroup):
    """
    In this type of grasp we get the intended object's pose from Gazebo directly, there is no object detection
    Currently only support grasping from top
    """
    def __init__(self) -> None:
        super(SimpleGraspPipeline, self).__init__()
        self.object = None

    def set_grasp_object(self, object : str):
        self.object = object

    def grasp(self, object_pose : Pose):
        """
        Perform grasp operation
        """

        # go to grasp position
        target_pose = self.transform_pose_to_top_pick(object_pose)
        target_pose.position.z += 0.7
        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        # go down and grasp
        waypoints = []
        self.set_gripper_open_pose()
        wpose = self.arm_group.get_current_pose().pose
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        self.arm_group.execute(plan, wait=True)
        self.set_gripper_close_pose()
        self.arm_group.clear_pose_targets()

    def put_in_box_right(self):
        """
        put the object to the box on the right
        """
        waypoints = []
        wpose = self.arm_group.get_current_pose().pose
        wpose.position.y -= 0.6
        waypoints.append(deepcopy(wpose))
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        self.arm_group.execute(plan, wait=True)
        self.set_gripper_open_pose()
        self.arm_group.clear_pose_targets()

    @staticmethod
    def transform_pose(object_pose : Pose) -> Pose:
        """
        Transform object_pose to target_pose in the world frame, useful when robot is not spawned at the origin
        """
        rospy.wait_for_service("/gazebo/get_model_state", 10.0)
        get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service('/gazebo/set_model_state', 10.0)
        set_pose_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        set_pose_srv.call('robot', )
        robot_pose = get_pose_srv.call('robot', "world").pose
        target_pose = deepcopy(object_pose)
        target_pose.position.x -= robot_pose.position.x
        target_pose.position.y -= robot_pose.position.y
        target_pose.position.z -= robot_pose.position.z
        target_pose.orientation.x -= robot_pose.orientation.x
        target_pose.orientation.y -= robot_pose.orientation.y
        target_pose.orientation.z -= robot_pose.orientation.z
        target_pose.orientation.w -= robot_pose.orientation.w
        
        return target_pose

    @staticmethod
    def gazebo_set_init_pose():
        """
        Robot would fly off with this method right now
        """
        state_msg = ModelState()
        state_msg.model_name = 'robot'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0.7071063
        state_msg.pose.orientation.w = 0.7071073

        rospy.wait_for_service("/gazebo/set_model_state", 10.0) 
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            rsp = set_state(state_msg)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

if __name__ == "__main__":

    rospy.init_node("simple_grasp_sim", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", 10.0)  
    get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    pipeline = SimpleGraspPipeline()

    pipeline.set_grasping_home_pose()
    pipeline.set_grasp_object('beer')
    object_pose = get_pose_srv.call(pipeline.object, "robot").pose
    box_pose = get_pose_srv.call("dropbox", "robot").pose
    pipeline.grasp(object_pose)
    pipeline.go_up()
    pipeline.put_in_box_right()

    rospy.spin()
