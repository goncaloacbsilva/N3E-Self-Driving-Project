#!/usr/bin/env python

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

def whiteColorFilter (frame):

    # It converts the BGR color space of image to HSV color space
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Threshold of white in HSV space
    lower_white = np.array([0, 0, 175])
    upper_white = np.array([255, 255, 255])

    mask_white = cv.inRange(hsv, lower_white, upper_white)

    return mask_white

def cannyFilter (frame):

    frameGray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frameBlur = cv.GaussianBlur(frameGray, (5,5) ,0)
    frameEdge = cv.Canny(frameBlur,100,200)

    return frameEdge

def regionOfInterest (frame):
    height = frame.shape[0]
    trapeze = np.array([
    [(500,height),(1300,height),(1100,600),(700,600)]
    ])
    mask = np.zeros_like(frame)
    cv.fillPoly(mask,trapeze,255)
    maskedFrame = cv.bitwise_and(frame,mask)
    return maskedFrame

def warp (frame):

    height = frame.shape[0]
    lenght = frame.shape[1]

#    Retangulo Final

    dst = np.float32([
    [(0,height),(lenght,height),(lenght,0),(0,0)]
    ])

#    Trapezio Inicial

    src =  np.float32([
    [(500,height),(1300,height),(1100,600),(700,600)]
    ])

    Z = cv.getPerspectiveTransform(src,dst)
    warp = cv.warpPerspective(frame,Z,(lenght,height))
    return warp


if __name__ == '__main__':

    frame = cv.imread('test_image.jpg',1)

    white = whiteColorFilter(frame)
    canny = cannyFilter(frame)
    whiteCanny = cv.bitwise_or(white,canny)
    PolymaskWhiteCanny = regionOfInterest(whiteCanny)
    warp = warp (PolymaskWhiteCanny)

    cv.imshow('yet',warp)
    cv.waitKey(0)

    cv.destroyAllWindows()
