#!/usr/bin/env python3.8
from amiga_group import AmigaMovegroup
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion, PoseStamped
from math import pi, cos , sin
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge.core import CvBridge
import tf2_ros
import rospy
import tf
import time
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal

class DataCollectionPipeline(AmigaMovegroup):
    def __init__(self) -> None:
        """
        Customly set layer, radius, longitude and latitude for your specific purposes
        """
        super(DataCollectionPipeline, self).__init__()
        self.sphere_layer = 2
        self.sphere_radius = 0.4
        self.longitude = 3
        self.latitude = 4
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_set = []
        self.pose_set = []
        self.name_set = []
        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose_goal", PlanToPose)

    def generate_sphere(self):
        """
        Generate a sphere of view points for the arm to reach for data collection purposes
        """
        # Centre              
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        #static_transformStamped.header.frame_id = 'zed2_left_camera_optical_frame'
        static_transformStamped.header.frame_id = 'zed2_left_camera_optical_frame'
        static_transformStamped.child_frame_id = "point_centre"
        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0
        quat = quaternion_from_euler(-0.15785, 0.63107, 0.6549)
        static_transformStamped.transform.rotation.x = -quat[0]
        static_transformStamped.transform.rotation.y = -quat[1]
        static_transformStamped.transform.rotation.z = -quat[2]
        static_transformStamped.transform.rotation.w = -quat[3]
        self.tf_set.append(static_transformStamped)
        
        # Sampling sphere coordinate
        for l in range(self.sphere_layer):
            for i in range(self.longitude):
                for j in range(self.latitude):
                    theta = pi * (90 / (self.latitude + 1)) * ((self.latitude - 1) / 2 - j) / 180
                    phi = pi * (360 / self.longitude / 2) * i / 180
                    x = self.sphere_layer * self.sphere_radius * sin(theta) * cos(phi)
                    y = self.sphere_layer * self.sphere_radius * sin(theta) * sin(phi)
                    z = self.sphere_layer * self.sphere_radius * cos(theta) - 1.2
                    roll = theta * sin(-phi)
                    pitch = theta * cos(phi)
                    yaw = 0
                    frame_label = "point " + str(l) + "," + str(i) + "," + str(j)
                    quat = quaternion_from_euler(roll, pitch, yaw)
                    static_transformStamped = TransformStamped()
                    static_transformStamped.header.stamp = rospy.Time.now()
                    static_transformStamped.header.frame_id = 'zed2_left_camera_optical_frame'
                    static_transformStamped.child_frame_id = frame_label
                    # static_transformStamped.transform.translation.x = -(x + 0.01757)
                    # static_transformStamped.transform.translation.y = -(y - 0.128)
                    # static_transformStamped.transform.translation.z = -(z - 0.1425)
                    static_transformStamped.transform.translation.x = -(x + 0.0525)
                    static_transformStamped.transform.translation.y = -(y + 0.1057)
                    static_transformStamped.transform.translation.z = -(z + 0.128)
                    static_transformStamped.transform.rotation.x = -quat[0]
                    static_transformStamped.transform.rotation.y = -quat[1]
                    static_transformStamped.transform.rotation.z = -quat[2]
                    static_transformStamped.transform.rotation.w = -quat[3]
                    self.pose_set.append(Pose(Vector3(-(x + 0.0525), -(y + 0.1057), -(z + 0.128)), Quaternion(-quat[0], -quat[1], -quat[2], -quat[3])))
                    self.name_set.append(frame_label)
                    self.tf_set.append(static_transformStamped)
    
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

count = 0
flag = False

# def callback_rgb(img):
#     if flag == True:
#         print("Saved")
#         cv2_img = CvBridge.imgmsg_to_cv2(img)
#         directory = r'/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images'
#         filename = 'savedImage{0}.jpg'.format(count)
#         os.chdir(directory)
#         cv2.imwrite(filename, cv2_img)

if __name__ == "__main__":

    rospy.init_node("data_collection_sim", anonymous=True)
    # rospy.Subscriber(
    #         "/l515_grip/color/image_raw",
    #         Image,
    #         callback_rgb
    #         )
    pipeline = DataCollectionPipeline()
    pipeline.generate_sphere()
    pipeline.br.sendTransform(pipeline.tf_set)
    
    pose_bl_list = []
    for pose in pipeline.pose_set:
        pose_bl = pipeline.transform_pose(input_pose=pose, from_frame="zed2_left_camera_optical_frame", to_frame="base_link")
        pose_bl_list.append(pose_bl)

    p = PoseStamped()
    p.header.frame_id = pipeline.robot.get_planning_frame()
    p.pose.position.x = 0.82
    p.pose.position.y = 0.15
    p.pose.position.z = -0.15
    pipeline.scene.add_box("workbench", p, (1.0, 1.0, 0.035))

    print(len(pose_bl_list))
    for i, target_pose in enumerate(pose_bl_list):
        if i % 3 == 0:
            res = pipeline.plan_to_pose_goal_srv.call(target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
            time.sleep(2)
        


    # while not rospy.is_shutdown():
    #     for name in pipeline.name_set:
    #         # static_transformStamped = TransformStamped()
    #         # static_transformStamped.header.stamp = rospy.Time.now()
    #         # static_transformStamped.header.frame_id = 'l515_grip_color_optical_frame'
    #         # static_transformStamped.child_frame_id = name + "base"
    #         try:
    #             trans = pipeline.tfBuffer.lookup_transform('base_link', name, rospy.Time())
    #             #tf_set_base_link.append(trans)
    #             tf_set_base_link.append(trans)
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             continue
    #         #print(tf_set_base_link)
    #         #pipeline.br.sendTransform(tf_set_base_link)
    #         point_3d = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    #         quaternion = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
    #         plan = pipeline.plan_pose_goal(point_3d, quaternion)
    #         if plan == True:
    #             print("Succeed")
    #             count += 1
    #     # print("finished, exited ...")
    #     # print("finished, exited ...")
    #     # print("finished, exited ...")
    #     # rospy.signal_shutdown(reason="closed")
    # #          flag = True



    #pipeline.set_grasping_home_pose()

    # for pose in pipeline.tf_set:
    # #     print(pose.transform)
    #     flag = False
    #     position_3d = [pose.trans.form]
    #     plan = pipeline.plan_pose_goal(pose=pose.transform)
    #     listener = tf.TransformListener()
    #     (trans,rot) = listener.lookupTransform('l515_grip_color_optical_frame', 'base_link', rospy.Time(0))
    #     if plan == True:
    #          print("Succeed")
    #          count += 1
    #          flag = True
        
            
    #print("{0} our of {1} cases succeeded".format(count, len(pipeline.pose_set)))

    rospy.spin()
