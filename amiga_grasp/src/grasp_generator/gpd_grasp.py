#!/usr/bin/env python3.8
#from asyncio.windows_utils import pipe
from turtle import pos
import rospy
import actionlib
import os
import yaml
import time
import open3d as o3d
import tf2_ros
import numpy as np
import open3d as o3ds
import math
from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from sensor_msgs.msg import PointCloud2, Image
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, PoseStamped
from moveit_task_constructor_gpd.srv import PointCloud
import moveit_commander
import moveit_msgs.msg
import rospy
import sys
import tf
import numpy as np
import tf2_ros
import struct
import os
import moveit_msgs.msg
from copy import deepcopy
from geometry_msgs.msg import Pose, Vector3, TransformStamped
from typing import Union, List, Tuple, Optional
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, rotation_matrix
from amiga_group import AmigaMovegroup
from numpy.linalg import inv
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal


class GPDGraspPipeline(AmigaMovegroup):
    def __init__(self) -> None:
        super(GPDGraspPipeline, self).__init__()

        rospy.init_node("gpd_grasp_rr", anonymous=True)
        rospy.Subscriber("/generate_grasps/feedback", SampleGraspPosesActionFeedback, self.callback_gpd)
        rospy.wait_for_service("/save_point_cloud", 10.0)
        rospy.wait_for_service("/amiga_gripper/init_gripper", 10.0)
        rospy.wait_for_service("/amiga_gripper/open_gripper", 10.0)
        rospy.wait_for_service("/amiga_gripper/close_gripper", 10.0)
        rospy.wait_for_service("/amiga_gripper/pinch_mode_gripper", 10.0)
        rospy.wait_for_service("/amiga_gripper/basic_mode_gripper", 10.0)
        self.collect_pc = rospy.ServiceProxy("/save_point_cloud", PointCloud)
        self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.gripper_set_pinch_mode = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.gripper_set_basic_mode = rospy.ServiceProxy("/amiga_gripper/basic_gripper", Empty)
        self.gripper_set_scissor_mode = rospy.ServiceProxy("/amiga_gripper/scissor_gripper", Empty)
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose_goal", PlanToPose)
        self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.gripper_wide_mode = rospy.ServiceProxy("/amiga_gripper/wide_mode_gripper", Empty)
        self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joint_goal_srv = rospy.ServiceProxy('/amiga/offline_manipulation/plan_joints', PlanJointGoal)
        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)

        #self.gripper_init()
        # self.gripper_set_scissor_mode.call()
        self.client = actionlib.SimpleActionClient('generate_grasps', SampleGraspPosesAction)
        self.grasp_pose = None
        self.trial = 0


    def callback_gpd(self, data):
        self.grasp_pose = data.feedback.grasp_candidates[0].pose
        print(self.grasp_pose)

    def grasp_detection(self):
        print("waiting for server ...")
        self.client.wait_for_server()
        print("server connected")
        goal = SampleGraspPosesGoal()
        self.client.send_goal(goal)
        time.sleep(2)
        self.client.wait_for_result()
        return self.client.get_result()    

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
        print("tf of target link successfully sent")
    
    @staticmethod
    def crop_filter(cloud, min_x=0, max_x=math.inf, min_y=0, max_y=math.inf, min_z=1, max_z=math.inf):

        points = np.asarray(cloud.points)

        ind = np.where((points[:, 0] >= min_x) & (points[:, 0] <= max_x) &
                    (points[:, 1] >= min_y) & (points[:, 1] <= max_y) &
                    (points[:, 2] >= min_z) & (points[:, 2] <= max_z))[0]

        inlier_cloud = cloud.select_by_index(ind)
        outlier_cloud = cloud.select_by_index(ind, invert=True)
        return inlier_cloud, outlier_cloud


    def process_cloud(self, InPath):
        pcd = o3d.io.read_point_cloud(InPath)
        in_box_cloud, out_box_cloud = self.crop_filter(pcd,
                                            min_x=0.5, max_x=1.0,
                                            min_y=-0.3, max_y=0.1,
                                            min_z=-0.1, max_z=0.1) 
        o3d.visualization.draw_geometries([in_box_cloud])

        # return in_box_cloud

        plane_model, inliers = in_box_cloud.segment_plane(distance_threshold=0.02,
                                                ransac_n=3,
                                                num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = in_box_cloud.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = in_box_cloud.select_by_index(inliers, invert=True)
        #outlier_cloud.paint_uniform_color([0, 1, 0])
        print(inlier_cloud)   # 位于方框内的点云点的个数
        print(outlier_cloud)  # 位于方框外的点云点的个数
        #o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
        return outlier_cloud

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

    @staticmethod
    def transform_pose(input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(2))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            raise e

    @staticmethod
    def rotation_to_amgia():
        q = quaternion_from_euler(0, math.pi/2, 3*math.pi/2)
        tf = [0, 0, 0, q[0], q[1], q[2], q[3]]
        pipeline.static_tf_broadcast('grip_baselink', "amiga_grip_baselink", tf)


if __name__ == "__main__":

    pipeline = GPDGraspPipeline()

    os.chdir('/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/tmp')
    InPath = "tmp.pcd"
    OutPath = "tmp_processed.pcd"
    
    while not rospy.is_shutdown():

        #build grasp frame ...
        # pipeline.collect_pc("tmp.pcd")
        # time.sleep(2)
        # processed_cloud = pipeline.process_cloud(InPath)
        # o3d.io.write_point_cloud(OutPath, processed_cloud)
        # o3d.visualization.draw_geometries([processed_cloud])
        # print("save succeed")
        
        result = pipeline.grasp_detection()
        feedback = rospy.wait_for_message("/generate_grasps/feedback", SampleGraspPosesActionFeedback)
        time.sleep(5)
        if result.grasp_state == "success":
            #print(feedback)
            #target_pose = feedback.grasp_candidates[0].pose

            target_pose = pipeline.grasp_pose
            print(target_pose.position.x)
            print(target_pose.position.y)
            print(target_pose.position.z)
            print(target_pose.orientation.x)
            print(target_pose.orientation.y)
            print(target_pose.orientation.z)
            print(target_pose.orientation.w)
        else:
            continue

        
        # os.remove(InPath) 
        # os.remove(OutPath)
        pose_in_list = [target_pose.position.x, target_pose.position.y, target_pose.position.z, 
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]

        # pipeline.static_tf_broadcast('zed2_left_camera_frame', "grasp_link_camera", pose_in_list)


        r, p, y = euler_from_quaternion((target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w))
        gpd = quaternion_matrix(quaternion_from_euler(r, p, y))
        r33 = quaternion_matrix(quaternion_from_euler(5*math.pi/4, math.pi/2, 3*math.pi/2))
        _q = quaternion_from_matrix(np.dot(gpd, r33))
        print(_q)

        target_pose.orientation.x = _q[0]
        target_pose.orientation.y = _q[1]
        target_pose.orientation.z = _q[2]
        target_pose.orientation.w = _q[3]

        # pose_in_list = pose_in_list[:3] + list(quaternion_from_euler(*pose_in_list[3:]))

        target_pose_bl = pipeline.transform_pose(target_pose, 'zed2_left_camera_frame', 'base_link')

        r_bl, p_bl, y_bl = euler_from_quaternion((target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w))
        r33 = quaternion_matrix(quaternion_from_euler(r_bl, p_bl, y_bl))
        r33 = np.delete(r33, 3, 0)
        r33 = np.delete(r33, 3, 1)
        dxdydz = np.dot(r33, np.array([0, 0, 0.10]))
        x, y, z = [target_pose_bl.position.x, target_pose_bl.position.y, target_pose_bl.position.z]
        _x = x - dxdydz[0]
        _y = y - dxdydz[1]
        _z = z - dxdydz[2]       

        pose_in_list_bl = [target_pose_bl.position.x, target_pose_bl.position.y, target_pose_bl.position.z, 
             target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]
        pipeline.static_tf_broadcast('base_link', "grip_baselink", pose_in_list_bl)
        pose_in_list_eef = [_x, _y, _z, 
            target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]
        pipeline.static_tf_broadcast('base_link', "eef_baselink", pose_in_list_eef)


        # translation, rotation = [0, 0, 0.25], [target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]
        # eef2vir = np.dot(tf.transformations.translation_matrix(translation), tf.transformations.quaternion_matrix(rotation)) # build a 3x4 3D affine matrix
        # vir2eef = inv(eef2vir)
        # pose_3d = [target_pose_bl.position.x, target_pose_bl.position.y, target_pose_bl.position.z]
        # pose_3d = np.dot(vir2eef, np.array(pose_3d+[1]))
        # pose_in_list_eef = [pose_3d[0], pose_3d[1], pose_3d[2], 
        #     target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]
        # pipeline.static_tf_broadcast('base_link', "eef_baselink", pose_in_list_eef)
        p = PoseStamped()
        p.header.frame_id = pipeline.robot.get_planning_frame()
        p.pose.position.x = 0.82
        p.pose.position.y = 0.16
        p.pose.position.z = -0.15
        pipeline.scene.add_box("workbench", p, (1.0, 1.0, 0.035))

        waypoints = []
        wpose = pipeline.arm_group.get_current_pose().pose
        wpose.position.x = _x
        wpose.position.y = _y
        wpose.position.z = _z
        wpose.orientation.x = target_pose_bl.orientation.x
        wpose.orientation.y = target_pose_bl.orientation.y
        wpose.orientation.z = target_pose_bl.orientation.z
        wpose.orientation.w = target_pose_bl.orientation.w
        waypoints.append(deepcopy(wpose))


        pipeline.arm_group.set_goal_orientation_tolerance(0.03)
        (plan, fraction) = pipeline.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        plan.joint_trajectory.header.frame_id="base_link"
        #print(plan)



        # pipeline.arm_group.execute(plan, wait=True)
        # pipeline.arm_group.clear_pose_targets()
        print(wpose)

        time.sleep(3)
        pipeline.gripper_close.call()
        time.sleep(3)
        pipeline.go_to_grasp_home_srv.call()


        

        # r, p, y = euler_from_quaternion((target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w))
        # r += (5*math.pi/4) % (2*math.pi)
        # p += (math.pi/2) % (2*math.pi)
        # y += (3*math.pi/2) % (2*math.pi)

        # q = quaternion_from_euler(r, p, y)
        # pose_in_list_bl[3] = q[0]
        # pose_in_list_bl[4] = q[1]
        # pose_in_list_bl[5] = q[2]
        # pose_in_list_bl[6] = q[0]

        # waypoints = []
        # wpose = pipeline.arm_group.get_current_pose().pose
        # waypoints.append(deepcopy(wpose))
        # wpose.position.x = target_pose_bl.position.x
        # wpose.position.y = target_pose_bl.position.y
        # wpose.position.z = target_pose_bl.position.z + 0.2
        # wpose.orientation.x = target_pose_bl.orientation.x
        # wpose.orientation.y = target_pose_bl.orientation.y
        # wpose.orientation.z = target_pose_bl.orientation.z
        # wpose.orientation.w = target_pose_bl.orientation.w
        # (plan, fraction) = pipeline.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        # plan.joint_trajectory.header.frame_id="base_link"
        # print(plan)
        # pipeline.arm_group.execute(plan, wait=True)
        # pipeline.arm_group.clear_pose_targets()

        # pose_in_list_bl[2] += 0.2
        # res = pipeline.plan_to_pose_goal_srv.call(pose_in_list_bl[0], pose_in_list_bl[1], pose_in_list_bl[2],
        #     pose_in_list_bl[3], pose_in_list_bl[4], pose_in_list_bl[5], pose_in_list_bl[6])
        
        #pipeline.rotation_to_amgia()
        #target_pose_bl_amiga = pipeline.transform_pose(target_pose_bl, 'grip_baselink', 'amiga_grip_baselink')

        # pose_in_list_bl_amiga = [target_pose_bl_amiga.position.x, target_pose_bl_amiga.position.y, target_pose_bl_amiga.position.z, 
        #     target_pose_bl_amiga.orientation.x, target_pose_bl_amiga.orientation.y, target_pose_bl_amiga.orientation.z, target_pose_bl_amiga.orientation.w]
        # pipeline.static_tf_broadcast('base_link', "final_grip", pose_in_list_bl_amiga)



        # pipeline.static_tf_broadcast('zed2_left_camera_frame', "grasp_link", pose_in_list)

        #position_3d = pipeline.camera_to_robot([target_pose.position.x, target_pose.position.y, target_pose.position.z], 'zed2_left_camera_frame', 'base_link')
        #target_pose_bl = Pose(position_3d, Quaternion(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w))

        # pose_in_list_bl = [target_pose_bl.position.x, target_pose_bl.position.y, target_pose_bl.position.z, 
        #     target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]

        # pipeline.static_tf_broadcast('base_link', "grasp_link_baselink", pose_in_list_bl)


        # waypoints = []
        # wpose = pipeline.arm_group.get_current_pose().pose
        # wpose.orientation.x = -0.5447976189748583
        # wpose.orientation.y = -0.013091948777348705
        # wpose.orientation.z = -0.009083614329181786
        # wpose.orientation.w = 0.8384161515543788
        
        # waypoints.append(deepcopy(wpose))
        # (plan, fraction) = pipeline.arm_group.compute_cartesian_path(
        #                             waypoints,   # waypoints to follow
        #                             0.01,        # eef_step
        #                             0.0)         # jump_threshold
        # plan.joint_trajectory.header.frame_id="base_link"
        # pipeline.arm_group.execute(plan, wait=True)
        # 
        # wpose3d = pipeline.camera_to_robot([wpose.position.x, wpose.position.y, wpose.position.z], 'amiga_arm_tool0', 'base_link')


        # x1, y1 = wpose3d.x, wpose3d.y
        # x2, y2 = target_pose_bl.position.x, target_pose_bl.position.y
        # z1, z2 = wpose3d.z, target_pose_bl.position.z
        # coeff = np.array([[z1, z2]]) @ inv(np.array([[x1, y1],[x2, y2]]))
        # inter_position_x = (x2-x1) * 0.7 + x1
        # inter_position_y = (y2-y1) * 0.7 + y1
        # print(coeff)
        # inter_position_z = coeff[0][0]*inter_position_x + coeff[0][1]*inter_position_y 

    
        # inter_postion = Vector3(inter_position_x, inter_position_y, inter_position_z)
        # print(type(inter_postion))

        # position_3d_scaled = Vector3(target_pose.position.x * 0.7, target_pose.position.y * 0.7, target_pose.position.z * 0.7)

        # position_3d_scaled = pipeline.camera_to_robot([position_3d_scaled.x, position_3d_scaled.y, position_3d_scaled.z], 'zed2_left_camera_frame', 'base_link')
        # target_pose_bl = Pose(position_3d, Quaternion(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w))

        # pose_in_list_bl = [position_3d_scaled.x, position_3d_scaled.y, position_3d_scaled.z, 
        #     target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]

        # pipeline.static_tf_broadcast('base_link', "grasp_link_scaled", pose_in_list_bl)





        # target_pose_inter = Pose(position_3d_scaled, Quaternion(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w))

        # pose_in_list_inter = [target_pose_inter.position.x, target_pose_inter.position.y, target_pose_inter.position.z, 
        #     target_pose_inter.orientation.x, target_pose_inter.orientation.y, target_pose_inter.orientation.z, target_pose_inter.orientation.w]

        # pipeline.static_tf_broadcast('base_link', "grasp_link_inter_baselink", pose_in_list_inter)


        # pipeline.plan_pose_goal(pose=pose_in_list_bl)
        
        #rospy.signal_shutdown(reason="end")

        
        rospy.spin()
