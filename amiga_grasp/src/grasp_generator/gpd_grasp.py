#!/usr/bin/env python3.8
import rospy
import actionlib
import os
import time
import open3d as o3d
import tf2_ros
import numpy as np
import math
from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped
from moveit_task_constructor_gpd.srv import PointCloud
import rospy
import tf
import numpy as np
import tf2_ros
import os
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped


class GPDGraspGenerator():
    def __init__(self) -> None:

        rospy.init_node("gpd_grasp_rr", anonymous=True)
        rospy.Subscriber("/generate_grasps/feedback", SampleGraspPosesActionFeedback, self.callback_gpd)

        self.collect_pc = rospy.ServiceProxy("/save_point_cloud", PointCloud)

        self.client = actionlib.SimpleActionClient('generate_grasps', SampleGraspPosesAction)
        self.grasp_pose = None

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
    def pose_to_list(pose):
        return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


if __name__ == "__main__":

    pipeline = GPDGraspGenerator()

    os.chdir('/home/yilong/git_ws/src/ur10e_robotiq/amiga_grasp/data/tmp')
    InPath = "tmp.pcd"
    OutPath = "tmp_processed.pcd"
    
    while not rospy.is_shutdown():

        # build grasp frame ...
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

        # pipeline.static_tf_broadcast('zed2_left_camera_frame', "grasp_link_camera", pipeline.pose_to_list(target_pose))

        r, p, y = euler_from_quaternion((target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w))
        gpd_target = quaternion_matrix(quaternion_from_euler(r, p, y))
        eef_gpd = quaternion_matrix(quaternion_from_euler(-5*math.pi/4, -math.pi/2, -3*math.pi/2))


        eef_target = quaternion_from_matrix(np.dot(eef_gpd, gpd_target))


        listener = tf.TransformListener()
        listener.waitForTransform("zed2_left_camera_frame", "amiga_arm_tool0", rospy.Time(), rospy.Duration(4.0))
        translation, rotation = listener.lookupTransform("zed2_left_camera_frame", "amiga_arm_tool0", rospy.Time(0))

        q = [rotation[0], rotation[1], rotation[2], rotation[3]]
        camera_eef = quaternion_matrix(q)
        camera_target = quaternion_from_matrix(np.dot(camera_eef, eef_target))

        target_pose.orientation.x = camera_target[0]
        target_pose.orientation.y = camera_target[1]
        target_pose.orientation.z = camera_target[2]
        target_pose.orientation.w = camera_target[3]


        target_pose_bl = pipeline.transform_pose(target_pose, 'zed2_left_camera_frame', 'base_link')     

        pose_in_list_bl = [target_pose_bl.position.x, target_pose_bl.position.y, target_pose_bl.position.z, 
             target_pose_bl.orientation.x, target_pose_bl.orientation.y, target_pose_bl.orientation.z, target_pose_bl.orientation.w]
        pipeline.static_tf_broadcast('base_link', "grasp_baselink", pose_in_list_bl)

        rospy.spin()
