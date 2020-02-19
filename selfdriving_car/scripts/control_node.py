#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.

import rospy
import time
import random
import socket
from std_msgs.msg import String

def callback(data):
    global velocity
    global pub
	v = velocity
    msg = str(v)+"/"+str(data.split("/")[1])
    rospy.loginfo(msg)
    pub.publish(msg)

def init_module():
    global velocity
    global pub
    pub = rospy.Publisher('Control_out', String, queue_size=10)
    rospy.init_node('control_node', anonymous=True)
    rospy.Subscriber("Control_in", String, callback)
    HOST = '0.0.0.0'
    PORT = 5000
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    print('[CONTROL NODE]: Waiting for connections at ' + str(HOST) + ":" + str(PORT))
    s.listen()
    conn, addr = s.accept()
    print('[CONTROL NODE]: Connected by', addr)
    while True:
        data = conn.recv(1024).decode("utf-8")
        if not data:
            break
        if data == "END":
            print('[CONTROL NODE]: Im shutting down...')
            break
        print('[CONTROL NODE]: Received command: ', data)
        velocity = data


if __name__ == '__main__':
    try:
        init_module()
    except rospy.ROSInterruptException:
        print("[CONTROL NODE]: Im shutting down...")
