#!/usr/bin/env python3.8
import rospy
import actionlib
import os
import time
import open3d as o3d
import tf2_ros
import numpy as np
import open3d as o3ds
import math
from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, Image
from simple_grasp_sim import SimpleGraspPipeline
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from moveit_task_constructor_gpd.srv import PointCloud
from cloud_processing import process_cloud


class GPDGraspPipeline():
    def __init__(self) -> None:
        super(GPDGraspPipeline, self).__init__()

        rospy.init_node("gpd_grasp_rr", anonymous=True)
        rospy.Subscriber("/generate_grasps/feedback", SampleGraspPosesActionFeedback, self.callback_gpd)
        #rospy.wait_for_service("/save_point_cloud", 10.0)
        #rospy.wait_for_service("/amiga_gripper/init_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/init_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/open_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/close_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/pinch_mode_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/basic_mode_gripper", 10.0)
        self.collect_pc = rospy.ServiceProxy("/save_point_cloud", PointCloud)
        # self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        # self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        # self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        # self.gripper_set_pinch_mode = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        # self.gripper_set_basic_mode = rospy.ServiceProxy("/amiga_gripper/basic_gripper", Empty)

        # self.gripper_init()
        self.client = actionlib.SimpleActionClient('generate_grasps', SampleGraspPosesAction)
        self.grasp_pose = None
        self.trial = 0

    def callback_gpd(self, data):
        self.grasp_pose = data.feedback.grasp_candidates[0].pose
        self.grasp_pose
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

if __name__ == "__main__":

    pipeline = GPDGraspPipeline()
    flag = True
    # pose_in_list = [0.8270310957793867, -0.1192083898563673, -0.0771259397005252, 
    # 0.6629045488071652, 0.6688486679314914, -0.3342576849807776, -0.038351253021636195]

    # # # #pose_in_list = [-0.2526103306973938, -0.2605980663175419, -0.2526103306973938, 
    # # # #               -0.26676472715905825, -0.5808220675640828, -0.11473647529134079, 0.7604721213904273]
    # pipeline.static_tf_broadcast('zed2_left_camera_frame', "grasp_link", pose_in_list)
    # rospy.spin()
    while flag:
        os.chdir('/home/yilong/git_ws/src/ur10e_robotiq/amiga_grasp/data/tmp')
        # InPath = "tmp.pcd"
        OutPath = "tmp_processed.pcd"
        # pipeline.collect_pc("tmp.pcd")
        # time.sleep(2)
        # processed_cloud = pipeline.process_cloud(InPath)
        # o3d.io.write_point_cloud(OutPath, processed_cloud)
        # o3d.visualization.draw_geometries([processed_cloud])
        # print("save succeed")
        result = pipeline.grasp_detection()
        #print(result.grasp_state)
        grasp = rospy.wait_for_message("/generate_grasps/feedback", SampleGraspPosesActionFeedback)
        print(grasp)

        # print(result)
        # target_pose = result.grasp_state
        # print(target_pose)
        # import ast
        # ast.literal_eval(target_pose)
        # pose_in_list = [0.13151713410088692, -0.9764348571180711, 0.05548434653209962, 
        #            -0.41605554522112564, -0.05637098044269422, -0.42635977282565013, 0.8012099849419884]
        # pipeline.static_tf_broadcast('zed2_left_camera_frame', "grasp_link", pose_in_list)
        # os.remove(InPath) 
        # os.remove(OutPath)
        pipeline.trial += 1
        flag = False
        rospy.spin()
