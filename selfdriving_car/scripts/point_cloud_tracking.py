#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import rospy as ros
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import pyzed.sl as sl
import sys

def main():

	rospy.init_node('pointCloudReciever', anonymous=True)
	rospy.Subscriber("/zed/zed_node/point_cloud/cloud_registered", PointCloud2, queue_size=10)

    while True:

        k = cv.waitKey(10)
        if k == 32:
            break

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
