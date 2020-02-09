#!/usr/bin/env python
import rospy
import serial_rsc.serial_lib as s_libs
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("[SERIAL_NODE]: Recieved String: " + str(data.data))
    rcv_msg = str(data.data)
    speed, angle = rcv_msg.split("/")
    speed = s_libs.convert_speed(speed)
    angle = s_libs.convert_slope(angle)
    rospy.loginfo("[SERIAL_NODE]: Velocity: " + str(speed) + " PWM / Angle: " + str(angle))

def module_init():
    rospy.loginfo("Initializing Serial Node...")
    rospy.init_node('serial_node', anonymous=True)
    rospy.Subscriber("Control_out", String, callback)
    rospy.spin()

if __name__ == '__main__':
    module_init()
