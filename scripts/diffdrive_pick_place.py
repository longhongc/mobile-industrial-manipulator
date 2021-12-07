##
# @file teleop.py
# @Brief Control the bot with wasdx key  
# @author Chang-Hong Chen
# @email longhongc@gmail.com
# @version 1.0.0
# @date 2021-10-23

import time
import signal
import sys
import rospy
import getch
import threading
from std_msgs.msg import Float64, Float64MultiArray

class DiffdriveArmTalker:

    def __init__(self):
        self.gripper1_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/gripper1_controller/command", Float64, queue_size=10)
        self.gripper2_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/gripper2_controller/command", Float64, queue_size=10)
        self.joint1_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint1_controller/command", Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint2_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint3_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint4_controller/command", Float64, queue_size=10)
        self.joint5_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint5_controller/command", Float64, queue_size=10)

        self.joints = [0, 0, 0, 0, 0]
        self.is_grasp=False
        self.rate = rospy.Rate(10)

    def pick(self):
        self.joints = [1.4, 0.3, 0.4, 0.15, -1.57]

    def reset(self):
        self.joints = [0, 0, 0, 0, 0]

    def lift(self):
        self.joints[2]-=0.4
        joint_value = Float64()
        joint_value.data = self.joints[2] 
        self.joint3_pub.publish(joint_value)

    def set_joints(self):
        rospy.loginfo("Move to grasp pose")
        joint_value = Float64()
        joint_value.data = self.joints[0] 
        self.joint1_pub.publish(joint_value)
        joint_value.data = self.joints[4] 
        self.joint5_pub.publish(joint_value)
        joint_value.data = self.joints[1] 
        self.joint2_pub.publish(joint_value)
        joint_value.data = self.joints[3] 
        self.joint4_pub.publish(joint_value)
        joint_value.data = self.joints[2] 
        self.joint3_pub.publish(joint_value)
        self.ungrasp()

    def pre_grasp(self):
        self.joints[2]+=0.4
        joint_value = Float64()
        joint_value.data = self.joints[2] 
        self.joint3_pub.publish(joint_value)
        time.sleep(1)


    def grasp(self):
        while(self.is_grasp):
            gripper_force = Float64()
            gripper_force.data = -3
            rospy.loginfo("Gripper Grasp")
            self.gripper1_pub.publish(gripper_force)
            self.gripper2_pub.publish(gripper_force)


    def ungrasp(self):
        gripper_force = Float64()
        gripper_force.data = 3
        rospy.loginfo("Gripper Release")
        self.gripper1_pub.publish(gripper_force)
        self.gripper2_pub.publish(gripper_force)



def signal_handler(sig, frame):
    print("")
    print("bye bye!")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("arm_talker_node", anonymous=True)
    diffdrive_arm_pub = DiffdriveArmTalker()
    grasp_thread = threading.Thread(target=diffdrive_arm_pub.ungrasp)
    grasp_thread.start()
    while not rospy.is_shutdown():
        # get user key input with enter
        val = getch.getch() 
        if(val=="g"):
            diffdrive_arm_pub.is_grasp=True
            diffdrive_arm_pub.pre_grasp()
            grasp_thread.join()
            grasp_thread = threading.Thread(target=diffdrive_arm_pub.grasp)
            grasp_thread.start()
        elif(val=="p"):
            diffdrive_arm_pub.pick()
        elif(val=="r"):
            diffdrive_arm_pub.reset()
        elif(val=="l"):
            diffdrive_arm_pub.lift()
        elif(val=="a"):
            diffdrive_arm_pub.pick()
            diffdrive_arm_pub.set_joints()
            time.sleep(1)
            diffdrive_arm_pub.grasp()  
            diffdrive_arm_pub.lift()  
        elif(val=="u"):
            diffdrive_arm_pub.is_grasp=False
            grasp_thread.join()
            grasp_thread = threading.Thread(target=diffdrive_arm_pub.ungrasp)
            grasp_thread.start()
        else:
            continue
        diffdrive_arm_pub.set_joints()
        time.sleep(0.1)


