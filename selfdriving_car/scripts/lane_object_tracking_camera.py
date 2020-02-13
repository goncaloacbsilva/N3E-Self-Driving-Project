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


def regionOfInterest(frame, x1_l, x1_r, x2_l, x2_r):
    height = frame.shape[0]
    lenght = frame.shape[1]

    vertices = np.array([[(x2_l * lenght, 1 * height), (x1_l * lenght, 0.6 * height),
                          (x1_r * lenght, 0.6 * height), (x2_r * lenght, 1 * height)]])
    intVertices = vertices.astype(int)

    print(*intVertices, sep=" , ")

    mask = np.zeros_like(frame)
    cv.fillPoly(mask, intVertices, 255)
    maskedFrame = cv.bitwise_and(frame, mask)
    cv.imshow('6', mask)
    return maskedFrame


def warp(frame, x1_l, x1_r, x2_l, x2_r):

    height = frame.shape[0]
    lenght = frame.shape[1]

#    final area

    dst = np.float32([[(0 * lenght, 1 * height), (0 * lenght, 0 * height),
                       (1 * lenght, 0 * height), (1 * lenght, 1 * height)]])

#    Inicial area

    src = np.float32([[(x2_l * lenght, 1 * height), (x1_l * lenght, 0.65 * height),
                       (x1_r * lenght, 0.65 * height), (x2_r * lenght, 1 * height)]])

    Z = cv.getPerspectiveTransform(src, dst)
    return cv.warpPerspective(frame, Z, (lenght, height))


def createTrackBar():
    cv.namedWindow("Trackbar")
    cv.resizeWindow("Trackbar", 300, 200)
    cv.createTrackbar("upperOffset", "Trackbar", 60, 100, onChangeTrackBar)
    cv.createTrackbar("lowerOffset", "Trackbar", 80, 100, onChangeTrackBar)


def onChangeTrackBar(value):
    global trackBar1Pos
    trackBar1Pos = cv.getTrackbarPos("upperOffset", "Trackbar")
    print(trackBar1Pos)
    global trackBar2Pos
    trackBar2Pos = cv.getTrackbarPos("lowerOffset", "Trackbar")
    print(trackBar2Pos)


def percentageConverter(frame):
    global trackBar1Pos
    global trackBar2Pos
    length = frame.shape[1]
    center = length / 2
    x1_offset = center * trackBar1Pos / 100.0
    x1_left = center - x1_offset
    x1_right = center + x1_offset

    x1_left = x1_left / length
    x1_right = x1_right / length

    x2_offset = center * trackBar2Pos / 100.0
    x2_left = center - x2_offset
    x2_right = center + x2_offset

    x2_left = x2_left / length
    x2_right = x2_right / length

    return x1_left, x1_right, x2_left, x2_right


def main():
    frame = cv.imread('Test_Image.jpeg', 1)

    createTrackBar()
    onChangeTrackBar(0)
    x1_l, x1_r, x2_l, x2_r = percentageConverter(frame)
    print(x1_l)
    print(x1_r)
    print(x2_r)
    print(x2_l)
    flag = true
    while flag = true:
        x1_l, x1_r, x2_l, x2_r = percentageConverter(frame)
        white = whiteColorFilter(frame)
        canny = cannyFilter(frame)
        whiteCanny = cv.bitwise_or(white, canny)
        warpFrame = warp(whiteCanny, x1_l, x1_r, x2_l, x2_r)
        cv.imshow('WaprFrame', warpFrame)
        cv.imshow('PolymaskWhiteCanny', whiteCanny)

    cv.destroyAllWindows()


if __name__ == '__main__':
    trackBar1Pos = 0
    trackBar2Pos = 0
    main()
