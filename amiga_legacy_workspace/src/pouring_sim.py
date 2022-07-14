#!/usr/bin/env python3.8
from copy import deepcopy
from simple_grasp_sim import SimpleGraspPipeline
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, PoseStamped

import rospy

class PourPipeline(SimpleGraspPipeline):
    """
    In this type of grasp we get the intended object's pose from Gazebo directly, there is no object detection
    Currently only support grasping from top
    """
    def __init__(self) -> None:
        super(SimpleGraspPipeline, self).__init__()
        self.object = None
    
    def grasp_side(self, object_pose : Pose):
        """
        Perform grasp operation
        """
        self.set_gripper_open_pose()
        # # go to grasp position
        target_pose = self.transform_pose_to_side_pick(object_pose)
        target_pose.position.z += 0.11
        target_pose.position.x -= 0.3
        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        #go front and grasp
        waypoints = []
        wpose = self.arm_group.get_current_pose().pose
        for i in range (2):
            wpose.position.x += 0.11
            waypoints.append(deepcopy(wpose))
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        self.arm_group.execute(plan, wait=True)
        grasping_group = "gripper"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box("amiga_arm_tool0", "coke_can", touch_links=touch_links)
        self.set_gripper_close_pose()
        self.arm_group.clear_pose_targets()

        wpose = self.arm_group.get_current_pose().pose
        for i in range (3):
            wpose.position.z += 0.03
            waypoints.append(deepcopy(wpose))
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        self.arm_group.execute(plan, wait=True)
        self.arm_group.clear_pose_targets()

        for i in range (3):
            wpose.position.y += 0.073
            waypoints.append(deepcopy(wpose))
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        self.arm_group.execute(plan, wait=True)
        self.arm_group.clear_pose_targets()

    def pour(self):
        joint_goal = self.arm_group.get_current_joint_values()
        for i in range(12):
            joint_goal[5] -= 0.015
            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.arm_group.go(joint_goal, wait=True)
            self.arm_group.stop()
        print("1")
        for i in range(5):
            joint_goal[5] += 0.04
            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.arm_group.go(joint_goal, wait=True)
            self.arm_group.stop()
        print("2")
        for i in range(12):
            joint_goal[5] -= 0.015
            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.arm_group.go(joint_goal, wait=True)
            self.arm_group.stop()
        print("3")

if __name__ == "__main__":

    rospy.init_node("simple_grasp_sim", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", 10.0)  
    get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    pipeline = PourPipeline()
        # add the workbench
    pipeline.scene.remove_world_object("workbench")
    table_pose = get_pose_srv.call("cafe_table", "robot").pose
    rospy.sleep(2)
    p = PoseStamped()
    p.header.frame_id = pipeline.robot.get_planning_frame()
    # add the workbench
    p.pose.position.x = table_pose.position.x
    p.pose.position.y = table_pose.position.y
    p.pose.position.z = table_pose.position.z + 0.77
    pipeline.scene.add_box("workbench", p, (0.75, 1.0, 0.035))

    # rospy.sleep(2)
    # p = PoseStamped()
    # p.header.frame_id = pipeline.robot.get_planning_frame()
    # # add the coke
    # coke_pose = get_pose_srv.call("coke_can", "robot").pose
    # p.pose.position.x = coke_pose.position.x
    # p.pose.position.y = coke_pose.position.y
    # p.pose.position.z = coke_pose.position.z + 0.07
    # pipeline.scene.add_cylinder("coke", p, 0.2, 0.02)

    pipeline.set_grasp_object('coke_can')
    object_pose = get_pose_srv.call(pipeline.object, "robot").pose
    print("object_pose")
    pipeline.grasp_side(object_pose)
    #pipeline.pour()

    rospy.spin()


    
