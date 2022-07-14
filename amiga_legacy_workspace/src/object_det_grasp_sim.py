#!/usr/bin/env python3.8
import rospy
import struct
import tf
import numpy as np
from sensor_msgs.msg import PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes
from simple_grasp_sim import SimpleGraspPipeline

datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}

class ObjDetGraspPipeline(SimpleGraspPipeline):
    """
    rgb resolution 1920*1280
    pc resolution 1024*768
    """
    def __init__(self) -> None:
        super(ObjDetGraspPipeline, self).__init__()

        rospy.init_node("grasp_sim", anonymous=True)
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.callback_darknet)

        self.xCenterObject = None
        self.yCenterObject = None

    def callback_darknet(self, data):
        probability = 0
        for box in data.bounding_boxes:
            if box.Class == self.object and box.probability > probability:
                probability = box.probability
                self.xCenterObject = int((box.xmax-box.xmin)/2) + box.xmin
                self.yCenterObject = int((box.ymax-box.ymin)/2) + box.ymin
                print("center_X: " + str(self.xCenterObject) + " center_Y: " + str(self.yCenterObject) )
                rospy.loginfo(
                    box.Class + ": " + 
                    "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
                        box.xmin, box.xmax, box.ymin, box.ymax
                    )
                )

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

if __name__ == "__main__":

    pipeline = ObjDetGraspPipeline()
    pipeline.object = "snowboard" 
    # pipeline.set_grasping_top_pose()
    # pipeline.set_gripper_open_pose()
    pc_msg = rospy.wait_for_message('/l515_grip/depth/color/points', PointCloud2)
    print("pc_width: ", pc_msg.width, "pc_height: ", pc_msg.height)
    detected_3dObject = pipeline.get_xyz([pipeline.xCenterObject, pipeline.yCenterObject], pc_msg)
    print(detected_3dObject)

    # if pipeline.xCenterObject is not None: # if detected intended object
    #     detected_xCenterObject= pipeline.xCenterObject
    #     detected_yCenterObject= pipeline.yCenterObject
    #     detected_3dObject = pipeline.get_xyz([detected_xCenterObject, detected_yCenterObject], pc_msg)
    #     if detected_3dObject is not None:
    #         detected_3dObjectRobotFrame = pipeline.camera_to_robot(detected_3dObject, 'l515_grip_color_optical_frame', 'base_link')
    #         pipeline.plan_cartesian_path(detected_3dObjectRobotFrame)
    #         pipeline.set_gripper_close_pose()
    #         pipeline.set_grasping_home_pose()

    rospy.spin()
