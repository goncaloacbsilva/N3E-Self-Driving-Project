#!/usr/bin/env python

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import pyzed.sl as sl
import sys


def whiteColorFilter(frame):

    # Convert the BGR color space of image to HSV color space

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
    mask = np.zeros_like(frame)
    cv.fillPoly(mask, intVertices, 255)
    maskedFrame = cv.bitwise_and(frame, mask)
    return maskedFrame


def birdView(frame, x1_l, x1_r, x2_l, x2_r):

    # Setup inicial variables

    global trackBar3Pos
    global trackBar4Pos
    global trackBar5Pos

    height = frame.shape[0]
    lenght = frame.shape[1]

    # Inicial points for bird view transform

    src = np.float32([[(x2_l * lenght + (trackBar5Pos - lenght / 2), trackBar3Pos), (x1_l * lenght + (trackBar5Pos - lenght / 2), trackBar4Pos),
                       (x1_r * lenght + (trackBar5Pos - lenght / 2), trackBar4Pos), (x2_r * lenght + (trackBar5Pos - lenght / 2), trackBar3Pos)]])

    # Final points for bird view transform

    dst = np.float32([[(0 * lenght, 1 * height), (0 * lenght, 0 * height),
                       (1 * lenght, 0 * height), (1 * lenght, 1 * height)]])

    birdTransford = cv.getPerspectiveTransform(src, dst)
    return cv.warpPerspective(frame, birdTransford, (lenght, height))


def createTrackBar(frame):

    cv.namedWindow("Trackbar")
#    cv.resizeWindow("Trackbar", 300, 200)
    cv.createTrackbar("upperOffset", "Trackbar", 15, 100, onChangeTrackBar)
    cv.createTrackbar("lowerOffset", "Trackbar", 85, 100, onChangeTrackBar)
    cv.createTrackbar("lowerY", "Trackbar",
                      frame.shape[0] - 2, frame.shape[0], onChangeTrackBar)
    cv.createTrackbar("upperY", "Trackbar",
                      frame.shape[0] - 300, frame.shape[0], onChangeTrackBar)
    cv.createTrackbar("offset", "Trackbar",
                      int(frame.shape[1] / 2), frame.shape[1], onChangeTrackBar)


def onChangeTrackBar(value):

    global trackBar1Pos
    trackBar1Pos = cv.getTrackbarPos("upperOffset", "Trackbar")
    global trackBar2Pos
    trackBar2Pos = cv.getTrackbarPos("lowerOffset", "Trackbar")
    global trackBar3Pos
    trackBar3Pos = cv.getTrackbarPos("lowerY", "Trackbar")
    global trackBar4Pos
    trackBar4Pos = cv.getTrackbarPos("upperY", "Trackbar")
    global trackBar5Pos
    trackBar5Pos = cv.getTrackbarPos("offset", "Trackbar")


def percentageConverter(frame):

    global trackBar1Pos
    global trackBar2Pos

    # Converts number from 0 - 100 to pixel coordenates

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


def getHist(frame):

    # Makes Histogram of the botton half of the frame

    hist = np.sum(frame[frame.shape[0] // 2:, :], axis=0)
    return hist


def findHistPeaks(hist):

    # Find the two peak is the histogram

    centerPoint = (hist.shape[0] // 2)
    leftPeak = np.argmax(hist[:centerPoint])
    rightPeak = np.argmax(hist[centerPoint:]) + centerPoint

    return leftPeak, rightPeak


def slidingBox(frame, leftPeak, rightPeak, numberOfBoxes, boxMargin):

    # Declaring necessary arrays

    leftBoxPixelsX = []
    leftBoxPixelsY = []
    tempLeftBoxPixelsX = []
    tempLeftBoxPixelsY = []

    rightBoxPixelsX = []
    rightBoxPixelsY = []
    tempRightBoxPixelsX = []
    tempRightBoxPixelsY = []

    # Getting some inicial values

    leftBoxCenter = leftPeak
    rightBoxCenter = rightPeak
    boxHeight = frame.shape[0] // numberOfBoxes

    # Start internal frame

    warp_zero = np.zeros_like(frame).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    # Loops that goes thourgh all the boxes

    for window in range(numberOfBoxes):

        # Determine X and Y coordenates of next couple of boxes

        lowY = frame.shape[0] - (window * boxHeight)
        upperY = lowY - boxHeight

        leftBoxLeftX = leftBoxCenter - boxMargin
        leftBoxRightX = leftBoxCenter + boxMargin

        rightBoxLeftX = rightBoxCenter - boxMargin
        rightBoxRightX = rightBoxCenter + boxMargin

        # Make sure the values down go beyond the window

        if upperY < 0:
            upperY = 0
        if lowY > frame.shape[0]:
            lowY = frame.shape[0]

        if leftBoxLeftX < 0:
            leftBoxLeftX = 0
        if leftBoxRightX > frame.shape[1]:
            leftBoxRightX = frame.shape[1]

        if rightBoxLeftX < 0:
            rightBoxLeftX = 0
        if rightBoxRightX > frame.shape[1]:
            rightBoxRightX = frame.shape[1]

        # Show boxes in the screen

        cv.rectangle(color_warp, (leftBoxLeftX, lowY), (leftBoxRightX, upperY),
                     (100, 255, 255), 1)
        cv.rectangle(color_warp, (rightBoxLeftX, lowY), (rightBoxRightX, upperY),
                     (100, 255, 255), 1)

        # creates array with all of the X and Y coordenates of the pixels in the box

        tempLeftBoxPixelsX.clear()
        tempLeftBoxPixelsY.clear()
        for y in range(upperY, lowY):
            for x in range(leftBoxLeftX, leftBoxRightX):
                if frame[y, x] != 0:
                    tempLeftBoxPixelsX.append(x)
                    tempLeftBoxPixelsY.append(y)
                    leftBoxPixelsX.append(x)
                    leftBoxPixelsY.append(y)

        tempRightBoxPixelsX.clear()
        tempRightBoxPixelsY.clear()
        for y in range(upperY, lowY):
            for x in range(rightBoxLeftX, rightBoxRightX):
                if frame[y, x] != 0:
                    tempRightBoxPixelsX.append(x)
                    tempRightBoxPixelsY.append(y)
                    rightBoxPixelsX.append(x)
                    rightBoxPixelsY.append(y)

        # Update center of next boxes, but there
        # needs to be at least 10 pixels in the box

        if len(tempLeftBoxPixelsX) > 10:
            leftBoxCenter = np.int(np.mean(np.array([tempLeftBoxPixelsX])))

        if len(tempRightBoxPixelsX) > 10:
            rightBoxCenter = np.int(np.mean(np.array([tempRightBoxPixelsX])))

    # Get all lines polynomes and determine average
    if len(leftBoxPixelsX) > 3 or len(leftBoxPixelsY) > 3:
        leftPoly = np.polyfit(leftBoxPixelsY, leftBoxPixelsX, 2)
        leftPoly[0] = np.mean(leftPoly[0])
        leftPoly[1] = np.mean(leftPoly[1])
        leftPoly[2] = np.mean(leftPoly[2])

        # Find line coordenates in order to draw Polinomial line

        leftPolyY = np.linspace(0, frame.shape[0] - 1, frame.shape[0])
        leftPolyX = leftPoly[0] * leftPolyY**2 + \
            leftPoly[1] * leftPolyY + leftPoly[2]
        leftPolyPoints = np.array(
            [np.transpose(np.vstack([leftPolyX, leftPolyY]))])

        cv.polylines(color_warp, np.int32([leftPolyPoints]), isClosed=False, color=(
            200, 255, 155), thickness=4)

        calculateCurvature(color_warp, leftPoly, leftPeak)

    if len(rightBoxPixelsX) > 3 or len(rightBoxPixelsY) > 3:
        rightPoly = np.polyfit(rightBoxPixelsY, rightBoxPixelsX, 2)
        rightPoly[0] = np.mean(rightPoly[0])
        rightPoly[1] = np.mean(rightPoly[1])
        rightPoly[2] = np.mean(rightPoly[2])

        # Find line coordenates in order to draw Polinomial line

        rightPolyY = np.linspace(0, frame.shape[0] - 1, frame.shape[0])
        rightPolyX = rightPoly[0] * rightPolyY**2 + \
            rightPoly[1] * rightPolyY + rightPoly[2]
        rightPolyPoints = np.array(
            [np.transpose(np.vstack([rightPolyX, rightPolyY]))])

        cv.polylines(color_warp, np.int32([rightPolyPoints]), isClosed=False, color=(
            200, 255, 155), thickness=4)

    # Add both images together

    frame = cv.cvtColor(frame, cv.COLOR_GRAY2RGB)
    frame = cv.addWeighted(frame, 0.7, color_warp, 0.8, 0)

    return frame


def calculateCurvature(frame, Polinome, Peak):

    # Determine distance that corresponds to one pixel

    distancePerPixelX = 15.0 / frame.shape[1]
    distancePerPixelY = 3.7 / frame.shape[0]

    # Find first and second derivative of Polinomial equacion

    D1 = (2 * Polinome[0] * Peak + Polinome[1]) * \
        (distancePerPixelX / distancePerPixelY)
    D2 = (2 * Polinome[0]) * (distancePerPixelX / distancePerPixelY)

    # Use the circle radius and curve angle

    curve = ((1 + D1 * D1)**(3 / 2)) / np.absolute(D2)
    angle = 18000 / (3.14 * curve)
    print(angle)

    return angle, curve


def drawLines(frame, x1_l, x1_r, x2_l, x2_r):

    global trackBar3Pos
    global trackBar4Pos
    global trackBar5Pos


    cv.line(frame, (int(x1_l * frame.shape[1] + (trackBar5Pos - frame.shape[1] / 2)), trackBar4Pos),(int(x2_l * frame.shape[1] + (trackBar5Pos - frame.shape[1] / 2)), trackBar3Pos), (0, 255, 255))

    cv.line(frame, (int(x1_r * frame.shape[1] + (trackBar5Pos - frame.shape[1] / 2)), trackBar4Pos),(int(x2_r * frame.shape[1] + (trackBar5Pos - frame.shape[1] / 2)), trackBar3Pos), (0, 255, 255))

    return frame


def main():

    # Magical camera setup made by the gods at ZED

    zed = sl.Camera()
    input_type = sl.InputType()
    if len(sys.argv) >= 2:
        input_type.set_from_svo_file(sys.argv[1])
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    zed.open(init)

    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width / 2
    image_size.height = image_size.height / 2
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    zed.grab(runtime)
    zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
    frame = image_zed.get_data()

    createTrackBar(frame)
    onChangeTrackBar(0)

    while True:
        zed.grab(runtime)
        zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
        frame = image_zed.get_data()
        x1_l, x1_r, x2_l, x2_r = percentageConverter(frame)
        white = whiteColorFilter(frame)
        canny = cannyFilter(frame)
        whiteCanny = cv.bitwise_or(white, canny)
        birdFrame = birdView(whiteCanny, x1_l, x1_r, x2_l, x2_r)
        hist = getHist(birdFrame)
        leftPeak, rightPeak = findHistPeaks(hist)
        final = slidingBox(birdFrame, leftPeak, rightPeak, 15, 100)
        frameWithLines = drawLines(frame, x1_l, x1_r, x2_l, x2_r)
        cv.imshow('OPENCV TEST2', frameWithLines)
        cv.imshow('OPENCV TEST1', final)
        k = cv.waitKey(30)
        if k == 32:
            break

    cv.destroyAllWindows()


if __name__ == '__main__':
    trackBar1Pos = 0
    trackBar2Pos = 0
    main()
