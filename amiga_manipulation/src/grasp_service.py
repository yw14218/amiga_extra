#!/usr/bin/env python3.8
import rospy
from std_srvs.srv import Empty, EmptyResponse


class GraspInterface:

    def __init__(self):
        self.command = Robotiq3FGripperRobotOutput()
        self.pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput)

        s_init = rospy.Service('/amiga_gripper/init_gripper', Empty, self.init_gripper)
        s_reset = rospy.Service('/amiga_gripper/reset_gripper', Empty, self.reset_gripper)
        s_close = rospy.Service('/amiga_gripper/close_gripper', Empty, self.close_gripper)
        s_open = rospy.Service('/amiga_gripper/open_gripper', Empty, self.open_gripper)


    def reset_gripper(self, req): 
        self.command = Robotiq3FGripperRobotOutput()
        self.command.rACT = 0
        self.pub.publish(self.command)
        return EmptyResponse()

    def init_gripper(self, req):
        self.command = Robotiq3FGripperRobotOutput()
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSPA = 255
        self.command.rFRA = 150

        self.pub.publish(self.command)
        return EmptyResponse()

    def open_gripper(self, req):
        self.command.rPRA = 0
        self.pub.publish(self.command)
        #print("open")
        return EmptyResponse()

    def close_gripper(self, req):
        self.command.rPRA = 255
        self.pub.publish(self.command)
        #print("close")
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('GripperServicesManager')

    grp = PRLGripperInterface()
    
    rospy.spin()