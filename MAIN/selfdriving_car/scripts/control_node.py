#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.

import rospy
import time
from std_msgs.msg import String
import PID as pid_module

def talker():
    pub = rospy.Publisher('controlOut', String, queue_size=10)
    rospy.init_node('control_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = str(v)+"/"+str(stearAng)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

def main():
    pid = pid_module.PID(P, I, D)
    pid.SetPoint = posicaodesejada
    pid.setSampleTime(1)
    pid.udpate (posicaocarro)
    stearAng = pid.output

P = 2.0
I = 4.0
D = 0.1
stearAng = 0.0
v = 2.0

posicaocarro=0
posiçaodesejada=1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        main()
