import time
import signal
import sys
import rospy
import threading
import queue


from std_msgs.msg import Float64, Float64MultiArray
from mim_kinematic import MIM_Arm

class DiffdriveArmTalker:
    state = ["Reset", "Pick", "Grasp", "Place", "Idle"]
    pick_pose = [1.4, 0.45, 0.6, 0.15, -1.57, 0]

    def __init__(self):
        self.gripper1_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/gripper1_controller/command", Float64, queue_size=10)
        self.gripper2_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/gripper2_controller/command", Float64, queue_size=10)
        self.joint1_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint1_controller/command", Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint2_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint3_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint4_controller/command", Float64, queue_size=10)
        self.joint5_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint5_controller/command", Float64, queue_size=10)
        self.joint6_pub = rospy.Publisher("/mim_diffdrive_arm/mim_arm/joint6_controller/command", Float64, queue_size=10)

        self.joints = [0, 0, 0, 0, 0, 0]
        self.current_state = DiffdriveArmTalker.state[0]
        self.mim_arm = MIM_Arm(self.joints)
        self.is_grasp = False
        self.grasp_thread = threading.Thread(target=self.gripper_release)
        self.grasp_thread.start()
        self.rate = rospy.Rate(20)

    def execute_state(self):
        if(self.current_state == "Reset"):
            self.reset()
        elif(self.current_state == "Pick"):
            self.pick()
        elif(self.current_state == "Grasp"):
            self.grasp()
        else:
            self.idle()
        self.current_state == "Idle"

    def execute_all(self):
        self.current_state == "Reset"
        self.reset()

        self.current_state == "Pick"
        self.pick()

        self.current_state == "Grasp"
        self.grasp()

        self.current_state == "Place"
        self.place()

        self.current_state == "Idle"
        self.idle()

    def set_joints(self):
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
        joint_value.data = self.joints[5] 
        self.joint6_pub.publish(joint_value)       
        rospy.sleep(0.1)

    def reset(self):
        print("State: Reset")
        self.joints = [0, 0, 0, 0, 0, 0]
        self.set_joints()

        if (self.is_grasp):
            self.is_grasp = False
            self.grasp_thread.join()
            self.grasp_thread = threading.Thread(target=self.gripper_release)
            self.grasp_thread.start()

    def pick(self):
        print("State: Pick")
        if(not self.joints == DiffdriveArmTalker.pick_pose):
            self.joints = DiffdriveArmTalker.pick_pose.copy()
            self.joints[2] = 0
            self._set_joints_duration(1)

            self.joints[2] = DiffdriveArmTalker.pick_pose[2] / 2
            self._set_joints_duration(1)

        self.joints = DiffdriveArmTalker.pick_pose
        self._set_joints_duration(2)

    def grasp(self):
        print("State: Grasp")
        length = 0.3
        velocity = -0.1
        simulated_points = 20
        joints_traj = self.mim_arm.grasp_motion(self.joints,
                length=length, velocity=velocity, simulated_points=simulated_points)

        joint_value = Float64()

        total_time = length / abs(velocity)
        delta_t = total_time / simulated_points

        for joints_config in joints_traj:
            self.joints = joints_config
            self._set_joints_duration(delta_t)

        self.is_grasp = True
        self.grasp_thread.join()
        self.grasp_thread = threading.Thread(target=self.gripper_grasp)
        self.grasp_thread.start()

        rospy.sleep(1)
        self.lift()

    def lift(self):
        print("State: Lift")
        length = 0.3
        velocity = 0.2
        simulated_points = 20
        joints_traj = self.mim_arm.grasp_motion(self.joints,
                length=length, velocity=velocity, simulated_points=simulated_points)

        joint_value = Float64()

        total_time = length / abs(velocity)
        delta_t = total_time / simulated_points

        for joints_config in joints_traj:
            self.joints = joints_config
            self._set_joints_duration(delta_t)

        self._set_joints_duration(1)


    def place(self):
        print("State: Place")
        self.joints = [-1.57, 0.45, 0, 0, -1.57, 0]
        self.set_joints()
        rospy.sleep(0.2)
        self.joints = [-1.57, 0.45, 0.6, 0.15, -1.57, 0]
        self.set_joints()

        rospy.sleep(0.5)

        length = 0.3
        velocity = -0.1
        simulated_points = 20
        joints_traj = self.mim_arm.grasp_motion(self.joints,
                length=length, velocity=velocity, simulated_points=simulated_points)

        joint_value = Float64()

        total_time = length / abs(velocity)
        delta_t = total_time / simulated_points

        for joints_config in joints_traj:
            self.joints = joints_config
            self._set_joints_duration(delta_t)


        self.is_grasp = False
        self.grasp_thread.join()
        self.grasp_thread = threading.Thread(target=self.gripper_release)
        self.grasp_thread.start()

        rospy.sleep(1)
        self.lift()

    def gripper_release(self):
        rospy.loginfo("Gripper Release")
        while not rospy.is_shutdown() and not self.is_grasp:
            gripper_force = Float64()
            gripper_force.data = 3
            self.gripper1_pub.publish(gripper_force)
            self.gripper2_pub.publish(gripper_force)
            rospy.sleep(0.1)

    def gripper_grasp(self):
        rospy.loginfo("Gripper Grasp")
        while not rospy.is_shutdown() and self.is_grasp:
            gripper_force = Float64()
            gripper_force.data = -18
            self.gripper1_pub.publish(gripper_force)
            self.gripper2_pub.publish(gripper_force)

    def idle(self):
        self.set_joints()

    def _set_joints_duration(self, duration):
        now = rospy.get_time()
        next_time = now + duration
        while now < next_time: 
            self.set_joints()
            now = rospy.get_time()
        

def add_input(input_queue):
    while True:
        input_queue.put(sys.stdin.read(1))

def print_start_menu():
    print("")
    print("Enter the following commands to control the arm: ")
    print("a) Execute Pick and Place")
    print("r) Reset the Robot to Initial State")
    print("p) Set the Robot to Pick State")
    print("g) Move Robot Downward and Grasp")
    print("Press Ctrl-c to exit")


def state_machine():
    print_start_menu()
    diffdrive_arm_pub = DiffdriveArmTalker()
    input_queue = queue.Queue()

    input_thread = threading.Thread(target=add_input, args=(input_queue,))
    input_thread.daemon = True
    input_thread.start()

    last_update = time.time()
    command = 'i'
    exit = False
    while not exit:

        if time.time()-last_update>0.5:
            if(command == 'a'):
                diffdrive_arm_pub.execute_all()
                print_start_menu()

            elif(command == 'r'):
                diffdrive_arm_pub.current_state = "Reset"
                diffdrive_arm_pub.execute_state()
                print_start_menu()
            elif(command == 'p'):
                diffdrive_arm_pub.current_state = "Pick"
                diffdrive_arm_pub.execute_state()
                print_start_menu()
            elif(command == 'g'):
                diffdrive_arm_pub.current_state = "Grasp"
                diffdrive_arm_pub.execute_state()
                print_start_menu()
            else:
                diffdrive_arm_pub.current_state = "Idle"
                diffdrive_arm_pub.execute_state()

            command = 'i'

            last_update = time.time()

        if not input_queue.empty() and not exit:
            command = input_queue.get()
            print("input:", input_queue.get())


def signal_handler(sig, frame):
    print("")
    print("bye bye!")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("arm_talker_node", anonymous=True)
    state_machine()

