import rospy
import ros_numpy
import json
import numpy as np
from sensor_msgs.msg import PointCloud2
import struct

datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}

def callback_pc(data):
    pc = ros_numpy.numpify(data)
    pass

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

# rgb resolution 1920*1280
# pc resolution 1024*768
Xmin = 1195
Xmax = 1848 
Ymin =  211
Ymax = 682
center_X = 1521 
center_Y = 446

rospy.init_node("pcl_test", anonymous=True)
rospy.Subscriber('/l515_grip/depth/color/points', PointCloud2, callback_pc)
pc_msg = rospy.wait_for_message('/l515_grip/depth/color/points', PointCloud2)

print(pc_msg.row_step) # 32768
print(pc_msg.point_step) # 32

print(pc_msg.fields[0]) 
print(pc_msg.fields[1])
print(pc_msg.fields[2])


xyz = get_xyz([1195, 211], pc_msg)
print(xyz)
# #pc = ros_numpy.numpify(pc_msg)
# # with open('pc.json', 'w', encoding ='utf8') as json_file:
# #     json.dump(pc.tolist(), json_file, allow_nan=True)
# m = np.array(json.load(open("pc.json")))

