#!/usr/bin/env python

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

def whiteColorFilter(frame):

    # It converts the BGR color space of image to HSV color space
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Threshold of white in HSV space
    lower_white = np.array([0, 0, 175])
    upper_white = np.array([255, 255, 255])

    mask_white = cv.inRange(hsv, lower_white, upper_white)

    return mask_white


def cannyFilter(frame):

    frameGray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frameBlur = cv.GaussianBlur(frameGray, (5, 5), 0)
    frameEdge = cv.Canny(frameBlur, 100, 200)

    return frameEdge


def regionOfInterest(frame):
    height = frame.shape[0]
    lenght = frame.shape[1]

    vertices = np.array([[(0.2 * lenght, 1 * height), (0.3 * lenght, 0.6 * height),
                         (0.7 * lenght, 0.6 * height), (0.8 * lenght, 1 * height)]])
    intVertices = vertices.astype(int)

    print(*intVertices, sep = " , ")

    mask = np.zeros_like(frame)
    cv.fillPoly(mask, intVertices, 255)
    maskedFrame = cv.bitwise_and(frame, mask)
    cv.imshow('6', mask)
    return maskedFrame


def warp(frame):

    height = frame.shape[0]
    lenght = frame.shape[1]

#    final area

    dst = np.float32([[(0 * lenght, 1 * height), (0 * lenght, 0 * height),
                     (1 * lenght, 0 * height), (1 * lenght, 1 * height)]])

#    Inicial area

    src = np.float32([[(0.2 * lenght, 1 * height), (0.3 * lenght, 0.6 * height),
                         (0.7 * lenght, 0.6 * height), (0.8 * lenght, 1 * height)]])

    Z = cv.getPerspectiveTransform(src, dst)
    return cv.warpPerspective(frame, Z, (lenght, height))

def createTrackBar():
    cv.namedWindow("Trackbar")
    cv.resizeWindow("Trackbar", 300, 200)
    cv.createTrackbar("upperOffset", "Trackbar",0 , 100, onChangeTrackBar)
    cv.createTrackbar("lowerOffset", "Trackbar",0 , 100, onChangeTrackBar)

def onChangeTrackBar(value):
    global trackBar1Pos = cv.getTrackbarPos("upperOffset", "Trackbar");
    global trackBar2Pos = cv.getTrackbarPos("lowerOffset", "Trackbar");

def percentageConverter ():



def main ():
    frame = cv.imread('Test_Image.jpeg', 1)

    white = whiteColorFilter(frame)
    canny = cannyFilter(frame)
    whiteCanny = cv.bitwise_or(white, canny)
    PolymaskWhiteCanny = regionOfInterest(whiteCanny)
    warpFrame = warp(PolymaskWhiteCanny)
    createTrackBar()

    cv.imshow('5', warpFrame)
    cv.waitKey(0)
    cv.destroyAllWindows()




if __name__ == '__main__':
    main()
