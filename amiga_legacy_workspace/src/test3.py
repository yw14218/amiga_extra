import os
import rospy
import tf
from PIL import Image
import numpy as np
# os.environ['ROS_MASTER_URI']="http://10.0.3.11:11311"
# rospy.init_node("gpd_grasp_rr", anonymous=True)

# listener = tf.TransformListener()
# listener.waitForTransform("base_link", "", rospy.Time(), rospy.Duration(4.0))
# translation, rotation = listener.lookupTransform(robot_link, camera_link, rospy.Time(0))
rgb="/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images/zed/rgb_7.png"
depth="/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images/zed/depth_7.png"
image1 = Image.open(rgb)
image2 = Image.open(depth)
(width, height) = image2.size
greyscale_map = list(image2.getdata())
greyscale_map = np.array(greyscale_map)
greyscale_map = greyscale_map.reshape((height, width))
print(greyscale_map)