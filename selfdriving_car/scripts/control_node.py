#!/usr/bin/env python3

import rospy
import time
import random
import socket
from std_msgs.msg import String
global pck_dat
velocity = 0
pck_dat = "0/3"

def callback(data):
    global pck_dat
    global pub
    msg = str(pck_dat)+"/"+str(data.data.split("/")[1])
    pub.publish(msg)

def init_module():
    global pck_dat
    global pub
    pub = rospy.Publisher('Control_out', String, queue_size=1)
    rospy.init_node('control_node', anonymous=True)
    rospy.Subscriber("Control_in", String, callback,queue_size=1)
    HOST = '0.0.0.0'
    PORT = 5000
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    rospy.loginfo('[CONTROL NODE]: Waiting for connections at ' + str(HOST) + ":" + str(PORT))
    s.listen(20)
    conn, addr = s.accept()
    print('[CONTROL NODE]: Connected by', addr)
    while True:
        data = conn.recv(1024).decode("utf-8")
        if not data:
            break
        if data == "END":
            rospy.loginfo('[CONTROL NODE]: Im shutting down...')
            break
        rospy.loginfo('[CONTROL NODE]: Received command: ' + data)
        rospy.loginfo('[CONTROL NODE]: Velocity: ' + str(data.split("/")[0]) + " Direction: " + str(data.split("/")[1]))
        pck_dat = data

if __name__ == '__main__':
    try:
        init_module()
    except rospy.ROSInterruptException:
        rospy.loginfo("[CONTROL NODE]: Im shutting down...")
