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
        self.joints = [1.42, 0.3, 0.35, 0.15, -1.52]

    def place(self):
        self.joints = [-1.57, -0.2, 1, 0.15, -1.57]

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
            gripper_force.data = -5
            rospy.loginfo("Gripper Grasp")
            self.gripper1_pub.publish(gripper_force)
            self.gripper2_pub.publish(gripper_force)

    def pre_place(self):
        self.joints[2]+=0.4
        joint_value = Float64()
        joint_value.data = self.joints[2] 
        self.joint3_pub.publish(joint_value)
        time.sleep(1)

    def ungrasp(self):
        gripper_force = Float64()
        gripper_force.data = 3
        rospy.loginfo("Gripper Release")
        self.gripper1_pub.publish(gripper_force)
        self.gripper2_pub.publish(gripper_force)

    def post_place(self):
        self.joints[2]-=0.4
        joint_value = Float64()
        joint_value.data = self.joints[2] 
        self.joint3_pub.publish(joint_value)
        time.sleep(1)



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
        # perform pick and place 
        diffdrive_arm_pub.pick()
        diffdrive_arm_pub.set_joints()
        time.sleep(1)
        diffdrive_arm_pub.is_grasp=True
        diffdrive_arm_pub.pre_grasp()  
        grasp_thread.join()
        grasp_thread = threading.Thread(target=diffdrive_arm_pub.grasp)
        grasp_thread.start()
        time.sleep(1)
        diffdrive_arm_pub.lift()  
        time.sleep(1)
        diffdrive_arm_pub.place()
        diffdrive_arm_pub.set_joints()
        time.sleep(1)
        diffdrive_arm_pub.pre_place()  
        diffdrive_arm_pub.is_grasp=False
        diffdrive_arm_pub.ungrasp()  
        time.sleep(0.5)
        diffdrive_arm_pub.post_place()  
        time.sleep(0.5)
        diffdrive_arm_pub.reset()  
        diffdrive_arm_pub.set_joints()

        print("Press r to restart or e to exit")
        val = getch.getch() 
        if(val=="r"):
            continue
            time.sleep(0.1)
        else:
            break


