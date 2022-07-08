import rospy
import tf2_ros
import numpy as np
import ros_numpy
import struct
import tf
import math
import time
from typing import Union
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, Image
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
from typing import List
from std_srvs.srv import Empty
from amiga_manip.srv import PlanCartesian, PlanToPose, PlanJointGoal

datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}

id_dict = {
    'bottle' : 44,
    'plate' : 26,# 45,
    'wine glass' : 46,
    'cup' : 47,
    'fork' : 48,
    'knife' : 49,
    'spoon' : 50,
    'bowl' : 51,
    'banana' : 52,
    'apple' : 53,
    'sandwich' : 54,
    'orange' : 55,
    'broccoli' : 56,
    'carrot' : 57,
    'hot dog' : 58,
    'pizza' : 59,
    'donut' : 60,
    'cake' : 61,
    'blender' : 83,
    'teddy bear' : 31 # 88 
} # interested classes in COCO dataset

class ObjDetGraspPipeline():
    def __init__(self) -> None:

        rospy.Subscriber("obj_detect/detectnet/detections", Detection2DArray, self.callback_detnet)
        rospy.Subscriber("/zed2_node/point_cloud/cloud_registered", PointCloud2, self.callback_pc)
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.callback_darknet)
        rospy.Subscriber("/zed2_node/depth/depth_registered", Image, self.callback_depth)
        #rospy.Subscriber("/zed2_node/rgb/image_rect_color", Image, self.callback_rgb)
        self.pub_pose = rospy.Publisher(
            '/amiga_grasp/obj_det/grasp_pose',
            Pose,
            queue_size=100
        )
        self.object = "teddy bear"
        self.grasp_link = "target link"
        self.grasp_pose = "top"
        self.plate = "plate"
        self.gripper_arm_offset = 0.15
        self.xCenterObject = None
        self.yCenterObject = None
        self.xplate = None
        self.yPlate = None
        self.pose1 = None
        self.xyzplate = None
        self.detected_3dObject = None
        self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        self.gripper_init.call()
        self.go_to_grasp_home_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_grasp_home_pose", Empty)

        self.inspect1_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_inspect1_pose", Empty)
        self.inspect2_srv = rospy.ServiceProxy("/amiga/offline_manipulation/go_to_inspect2_pose", Empty)

        self.plan_to_pose_goal_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_to_pose_goal", PlanToPose)
        self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        self.gripper_wide_mode = rospy.ServiceProxy("/amiga_gripper/wide_mode_gripper", Empty)
        self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        self.plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
        self.plan_joint_goal_srv = rospy.ServiceProxy('/amiga/offline_manipulation/plan_joints', PlanJointGoal)
        
    def callback_detnet(self, data):
        for detection in data.detections:
            if detection is not None and detection.results[0].id == id_dict[self.object]:
                self.xCenterObject = int(detection.bbox.center.x)
                self.yCenterObject = int(detection.bbox.center.y)
            if detection is not None and detection.results[0].id == id_dict[self.plate]:
                if self.xplate is None:
                    self.xplate = int(detection.bbox.center.x)
                    self.yplate = int(detection.bbox.center.y)
                
    def callback_darknet(self, data): 
        probability = 0
        for box in data.bounding_boxes:
            if box.Class == self.object and box.probability > probability:
                probability = box.probability
                self.xCenterObject = int((box.xmax-box.xmin)/2) + box.xmin
                self.yCenterObject = int((box.ymax-box.ymin)/2) + box.ymin

    def callback_depth(self, data): #(540, 940)
        data = ros_numpy.numpify(data)
        #print(self.yCenterObject, self.xCenterObject)
        #print(data[self.yCenterObject][self.xCenterObject]) # 0.62420475

    @staticmethod
    def convert_from_uvd(self, u, v, d):
        x_over_z = (self.cx - u) / self.focalx
        y_over_z = (self.cy - v) / self.focaly
        z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
        x = x_over_z * z
        y = y_over_z * z
        return x, y, z

    def callback_rgb(self, data): #(540, 940, 4)
        data = ros_numpy.numpify(data)
        #print("rgb:", data.shape)

    def callback_pc(self, data):
        if self.xCenterObject is not None and self.yCenterObject is not None:
            if self.detected_3dObject is None:
                detected_3dObject = self.get_xyz([self.yCenterObject, self.xCenterObject], data)
                for i in range(8):
                    for j in range(8):
                        if detected_3dObject == None:
                            detected_3dObject = self.get_xyz([self.yCenterObject + i, self.xCenterObject + j], data)
                if detected_3dObject is not None and math.isnan(detected_3dObject[0]) is False:
                    self.detected_3dObject = detected_3dObject
            print("bear: ", self.detected_3dObject)
            
        if self.xplate is not None and self.yplate is not None:
            if self.xyzplate is None:
                plate_3d = self.get_xyz([self.yplate, self.xplate], data)
                for i in range(7):
                    for j in range(7):
                        if plate_3d == None:
                            plate_3d = self.get_xyz([self.yplate + i, self.xplate + j], data)
                if plate_3d is not None and math.isnan(detected_3dObject[0]) is False:
                    self.xyzplate = plate_3d
            print("plate: ", self.xyzplate)
      		

    def start(self):
        while not rospy.is_shutdown():
            if self.detected_3dObject is not None and math.isnan(self.detected_3dObject[0]) is False:
                if self.xyzplate is not None and math.isnan(self.xyzplate[0]) is False: 
                    print("------------------------------------INFO-------------------------------------------")
                    print("3d object point:", pipeline.detected_3dObject)
                    detected_3dObjectRobotFrame = self.camera_to_robot(self.detected_3dObject, 'zed2_left_camera_frame', 'base_link')
                    target_pose = Pose(detected_3dObjectRobotFrame, Quaternion(0, 0, 0, 1))
                    
                    if self.grasp_pose == 'top':
                        target_pose = self.transform_pose_to_top_pick(target_pose)
                    elif self.grasp_pose == 'front':
                        target_pose = self.transform_pose_to_front_pick(target_pose)
                    else:
                        raise NotImplementedError

                    target_pose.position.z += self.gripper_arm_offset
                    print("target pose:", target_pose)
                    # send to tf
                    pose_in_list = [target_pose.position.x, target_pose.position.y, target_pose.position.z, 
                    target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
                    self.static_tf_broadcast('base_link', pipeline.grasp_link, pose_in_list)
                    self.pub_pose.publish(target_pose)
                    print(target_pose)

                    # grasp
                    self.grasp(target_pose)
                    self.go_to_grasp_home_srv.call()
                    
                    plate_3dObjectRobotFrame = self.camera_to_robot(self.xyzplate, 'zed2_left_camera_frame', 'base_link')
                    target_pose = Pose(plate_3dObjectRobotFrame, Quaternion(0, 0, 0, 1))
                    
                    if self.grasp_pose == 'top':
                        target_pose = self.transform_pose_to_top_pick(target_pose)
                    elif self.grasp_pose == 'front':
                        target_pose = self.transform_pose_to_front_pick(target_pose)
                    else:
                        raise NotImplementedError

                    target_pose.position.z += 0.3

                    print("finished, shutting down ...")
                    pose_in_list = [target_pose.position.x, target_pose.position.y, target_pose.position.z, 
                    target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
                    self.static_tf_broadcast('base_link', "plate_link", pose_in_list)
                    self.pub_pose.publish(target_pose)
                    
                   # res = self.plan_to_pose_goal_srv.call(target_pose.position.x, target_pose.position.y, target_pose.position.z,
                   #     target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
        # assert res is True
                    time.sleep(2)
                    self.gripper_open.call()
                    time.sleep(2)
                    self.go_to_grasp_home_srv.call()

                    rospy.signal_shutdown(reason="finished")


    def grasp(self, target_pose : Pose):
        """
        Perform grasp operation
        """
        res = self.plan_to_pose_goal_srv.call(target_pose.position.x, target_pose.position.y, target_pose.position.z,
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
        # assert res is True
        time.sleep(0.1)

        self.gripper_close.call()
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

    @staticmethod
    def transform_pose_to_front_pick(pose : Pose()) -> Pose:
        """
        setting an absolute orientation for grasp (from the front)
        """
        pose.orientation.x = 0.49956006551396753
        pose.orientation.y = 0.5004395699227152
        pose.orientation.z = 0.49991382218159247
        pose.orientation.w = 0.5000861407708072

        return pose

    @staticmethod
    def static_tf_broadcast(parent_id : str, child_id : str, pose_in_list : List) -> None:
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
    def get_xyz(point_2d, pc_msg):
        arrayPosition = point_2d[0]*pc_msg.row_step + point_2d[1]*pc_msg.point_step # point_2d: y,x
        pos_x = arrayPosition + pc_msg.fields[0].offset # X has an offset of 0
        len_x = datatype[pc_msg.fields[0].datatype]
        pos_y = arrayPosition + pc_msg.fields[1].offset # Y has an offset of 4
        len_y = datatype[pc_msg.fields[1].datatype]
        pos_z = arrayPosition + pc_msg.fields[2].offset # Z has an offset of 8
        len_z = datatype[pc_msg.fields[2].datatype]

        try:
            x = struct.unpack('f', pc_msg.data[pos_x: pos_x+len_x])[0] # read 4 bytes as a float number
            y = struct.unpack('f', pc_msg.data[pos_y: pos_y+len_y])[0]
            z = struct.unpack('f', pc_msg.data[pos_z: pos_z+len_z])[0]
            return [x,y,z]
        except:
            return None

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

if __name__ == "__main__":

    rospy.init_node("detnet_grasp", anonymous=True)
    pipeline = ObjDetGraspPipeline()
    pipeline.go_to_grasp_home_srv.call()
    #time.sleep(2.0)
    #pipeline.inspect1_srv.call()
    #time.sleep(2.0)
    #pipeline.inspect2_srv.call()
    #time.sleep(2.0)
    pipeline.go_to_grasp_home_srv.call()
    #pipeline.gripper_wide_mode.call()
    time.sleep(2.0)
    #pipeline.gripper_close.call()
    pipeline.start()
    
    
    #rospy.wait_for_service("/amiga/offline_manipulation/plan_cartesian_xyz", 10.0)
    #plan_cartesian_srv = rospy.ServiceProxy("/amiga/offline_manipulation/plan_cartesian_xyz", PlanCartesian)
    #pipeline.plan_cartesian_srv.call(0.0, 0.0, 0.3)

    rospy.spin()

