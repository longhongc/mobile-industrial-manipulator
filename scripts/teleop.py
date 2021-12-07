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
import getch
import rospy
from std_msgs.msg import Float64, Float64MultiArray

class SteeringTalker:

    def __init__(self):
        self.pub = rospy.Publisher("/newbot2/steering_controller/command", Float64, queue_size=10)
        self.rate = rospy.Rate(10)
        self.min_angle = -1
        self.max_angle = 1
        self.steering_angle = 0
    def update_key(self, key):
        # Left
        if(key=='A' or key=='a'):
            self.steering_angle += 0.2 
        # Right
        elif(key=='D' or key=='d'):
            self.steering_angle -= 0.2
        # Reset
        elif(key=='S' or key=='s'):
            self.steering_angle = 0.0

        self.steering_angle = self.clip_angle(self.steering_angle)

    def update_value(self, angle):
        self.steering_angle = self.clip_angle(angle)

    def send_msg(self):
        msg = Float64()
        msg.data = self.steering_angle
        rospy.loginfo("Steering Angle: %s", str(msg.data))
        self.pub.publish(msg)

    def clip_angle(self, angle):
        clipped = min(self.max_angle, angle)
        clipped = max(self.min_angle, clipped)
        return clipped
    
class DrivingTalker:

    def __init__(self):
        self.pub = rospy.Publisher("/newbot2/driving_controller_rear/command", Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(10)
        self.left_vel = 0
        self.right_vel = 0
        self.wheels_velocity = [self.left_vel, self.right_vel]
    def update_key(self, key):
        # Forward
        if(key=='W' or key=='w'):
            self.left_vel += 1
            self.right_vel += 1
        # Backward
        elif(key=='X' or key=='x'):
            self.left_vel -= 1
            self.right_vel -= 1
        # Stop
        elif(key=='S' or key=='s'):
            self.left_vel = 0.0
            self.right_vel = 0.0
        self.wheels_velocity = [self.left_vel, self.right_vel] 

    def update_value(self, vel):
        self.left_vel = vel[0]
        self.right_vel = vel[1]
        self.wheels_velocity = vel

    def send_msg(self):
        msg = Float64MultiArray()
        msg.data = self.wheels_velocity
        rospy.loginfo("Driving Speed: %s", str(msg.data))
        self.pub.publish(msg)


        
def parse_input(input_key, steering_pub, driving_pub):
    steering_keys=['A', 'a', 'D', 'd', 'S', 's']
    driving_keys=['W', 'w', 'X', 'x', 'S', 's']

    if(input_key in steering_keys):
        steering_pub.update_key(input_key)
        steering_pub.send_msg()

    if(input_key in driving_keys):
        driving_pub.update_key(input_key)
        driving_pub.send_msg()

def signal_handler(sig, frame):
    print("")
    print("bye bye!")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("teleop_talker", anonymous=True)
    steering_pub = SteeringTalker()
    driving_pub = DrivingTalker()
    while not rospy.is_shutdown():
        # get user key input with enter
        val = getch.getch() 
        parse_input(val, steering_pub, driving_pub)
        time.sleep(0.1)


