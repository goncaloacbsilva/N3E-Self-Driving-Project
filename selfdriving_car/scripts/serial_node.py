#!/usr/bin/env python
import rospy
import serial_lib as s_libs
import CarControl
from std_msgs.msg import String
global car

def callback(data):
    global car
    rospy.loginfo("[SERIAL_NODE]: Recieved String: " + str(data.data))
    rcv_msg = str(data.data)
    speed, direction, angle = rcv_msg.split("/")
    #speed = s_libs.convert_speed(speed) Right now we write the PWM value
    angle = s_libs.convert_slope(angle)
    rospy.loginfo("[SERIAL_NODE]: Velocity: " + str(speed) + " PWM / Direction: "+ str(direction) + " CODE / Angle: " + str(angle) + " PWM")
    car.set_speed(speed)
    car.set_direction(direction)
    car.set_steering(angle)
    car.update()

def module_init():
    global car
    rospy.loginfo("Initializing Serial Node...")
    car = CarControl.Car();
    rospy.init_node('serial_node', anonymous=True)
    rospy.Subscriber("Control_out", String, callback)
    rospy.spin()

if __name__ == '__main__':
    module_init()
