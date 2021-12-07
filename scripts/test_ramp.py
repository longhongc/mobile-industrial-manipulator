import time
import signal
import sys
import rospy
import getch
import threading
from geometry_msgs.msg import Twist

def main():
    print("start ramp testing")
    rospy.init_node("test_ramp_node", anonymous=True)
    diffdrive_pub = rospy.Publisher("/mim_diffdrive_arm/mim_diffdrive/mobile_base_controller/cmd_vel", Twist, queue_size=10)
    rockerb_pub = rospy.Publisher("/mim_rockerb_arm/mim_rockerb/mobile_base_controller/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)

    diffdrive_vel = Twist()
    diffdrive_vel.linear.x = -10
    diffdrive_vel.linear.y = 0
    diffdrive_vel.linear.z = 0
    diffdrive_vel.angular.x = 0
    diffdrive_vel.angular.y = 0
    diffdrive_vel.angular.z = 0

    rockerb_vel = Twist()
    rockerb_vel.linear.x = 10
    rockerb_vel.linear.y = 0
    rockerb_vel.linear.z = 0
    rockerb_vel.angular.x = 0
    rockerb_vel.angular.y = 0
    rockerb_vel.angular.z = 0

    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        diffdrive_pub.publish(diffdrive_vel)
        rockerb_pub.publish(rockerb_vel)
        t1 = rospy.Time.now().to_sec()
        if(t1-t0) > 5:
            print("finish")
            break
        rate.sleep()

def signal_handler(sig, frame):
    print("")
    print("bye bye!")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()

