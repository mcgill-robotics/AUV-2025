#!/usr/bin/env python3
import rospy
import numpy as np 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import sys

#TODO #1: Incorporate into ros
#TODO #2: Deal with noise or foreign objects
#TODO #3: Deal with case where we don't find lines

BRIGHTNESS_THRESOLD = 50 # Threshold used to compare grayscale image
CANNY_THRESHOLD_1 = 50 # Value for canny edge detection - used for edge linking
CANNY_THRESHOLD_2 = 255 # Used to find initial segments of strong edges
HOUGH_RHO = 6 # Distance resolution of accumulator in pixels
HOUGH_THETA = np.pi/180 # Angle resolution of accumulator in rad
HOUGH_THRESHOLD = 100 # Accumulator threshold (ie- mumber of votes) - DECREASE FOR MORE LINES
ANGLE_TOLERANCE = 0.4 # Tolerance for angles of considered to belong to the same line 

bridge = CvBridge() # Create CvBridge object to process ros image

def image_callback(ros_img):
    try:
        original = bridge.imgmsg_to_cv2(ros_img, 'bgr8') # Convert to opencv form
    except CvBridgeError as e:
        rospy.loginfo('Topic does not exist')
    #original = cv2.imread('source.png') # Initially using static image
    masked = original.copy() # Create copy of original
    masked[:,:,0:2] = 0 # Keep only red in the image
    gray = cv2.cvtColor(masked,cv2.COLOR_BGR2GRAY) # Convert to gray(red points will be brighter)
    gray = (gray > BRIGHTNESS_THRESOLD) * gray # Zero out parts  of image that are below thresold
    canny = cv2.Canny(gray, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2) # Perform Canny edge detection on grayscale image to find outlines
    hough = original.copy() # Create copy of original for final display of lines
    houghLines = cv2.HoughLines(canny,HOUGH_RHO,HOUGH_THETA,HOUGH_THRESHOLD) # Run Hough lines to detect straight lines

    '''
    NOTE: Houghlines returns lines in the form of rho and theta, which are the distance and angle respectively.
    The angle is returned in the range of 0 to 180 degrees(0 to pi in radians), measured clockwise from the x-axis to the NORMAL of the line. 
    This range (0 to 180 degrees) represents every possible slope of a line. The value of rho is positive for lines starting at the bottom left and moving
    towards the top right, and negative for the opposite case. To calrify the value of theta, draw the normal to the line you have, starting at the origin.
    Measure the clockwise angle from the positive axis to this line.
    https://docs.opencv.org/4.x/d6/d10/tutorial_py_houghlines.html
    '''

    if houghLines is not None:
        linesnp = np.array(houghLines[:,0]) # Lines in numpy array for vectorized mathematical operations
        rhos = linesnp[:,0] # All the rho values from the array
        thetaFromVertical = linesnp[:,1] # Values of theta
        thetaFromVertical[rhos<0] = np.pi-thetaFromVertical[rhos<0] # Deal with lines which have negative slope
        fromRef = np.min(thetaFromVertical) # Find point closest to vertical, assuming that we will be nearly aligned with this line
        toRef = np.max(thetaFromVertical) # Find point farthest from the vertical
        linesFrom = houghLines[thetaFromVertical<fromRef+ANGLE_TOLERANCE] # All lines close to vertical
        linesTo = houghLines[thetaFromVertical>toRef-ANGLE_TOLERANCE] # All lines far from vertical
        lineFromAvg = np.average(linesFrom,axis=0) # Take average for both lines
        lineToAvg = np.average(linesTo,axis=0)
        linesAvg = np.array([lineFromAvg,lineToAvg]) # Create array with the average rho and theta values

    color = (0,0,255)  # Set color to red initially, to represent 'from' lane
    rho = linesAvg[:,0,0]
    theta = linesAvg[:,0,1]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho # rho = x.cos(theta) + y.sin(theta)
    y0 = b * rho
    grad = -a/b # convert to y = mx+c OR y = [rho-x.cos(theta)]/sin(theta). Gradient is -cos/sin = -cot(theta)
    yint = rho / b # y = [rho-x.cos(theta)]/sin(theta), so the y-intercept or c is rho/sin(theta)
    #Lines for plotting taken from Opencv houghlines docs
    pt1 = (int(x0[0] + 1000*(-b[0])), int(y0[0] + 1000*(a[0])))
    pt2 = (int(x0[0] - 1000*(-b[0])), int(y0[0] - 1000*(a[0])))
    cv2.line(hough, pt1, pt2, color, 2, cv2.LINE_AA) # Plot line

    color = (0,255,0) # Change color to  green for 'to' line
    pt3 = (int(x0[1] + 1000*(-b[1])), int(y0[1] + 1000*(a[1])))
    pt4 = (int(x0[1] - 1000*(-b[1])), int(y0[1] - 1000*(a[1])))
    cv2.line(hough, pt3, pt4, color, 2, cv2.LINE_AA) # Plot line

    centerX = (yint[1]-yint[0])/(grad[0]-grad[1]) # Caculate the intersection of the lines, where m1x + c1 = m2X + c2
    centerY = centerX * grad[0] + yint[0] # y = mx + c
    cv2.circle(hough,(int(centerX),int(centerY)),10,color=(255,0,150),thickness=3) # Draw a circle at the center

    headings = theta # Initial headings
    headings[rho<0] = headings[rho<0]-np.pi # Conversion for certain slopes
    headings = headings * 180 /np.pi # Convert headings to degrees
    print('Heading from: ',headings[0])
    print('Heading to: ',headings[1])
    print('Center at: ',centerX,', ',centerY)


    # Only display
    cv2.imshow('Original',original)
    #cv2.imshow('Masked',masked)
    #cv2.imshow('Gray',gray)
    cv2.imshow('Canny',canny)
    cv2.imshow('Hough',hough)
    cv2.waitKey(5)


if __name__=='__main__':
    rospy.init_node('lane_marker',anonymous=True)
    cv2.namedWindow('Original',1)
    cv2.namedWindow('Canny',1)
    cv2.namedWindow('Hough',1)
    cv2.startWindowThread()
    image_sub = rospy.Subscriber('/image_publisher_1658414248915864811/image_raw',Image,image_callback)
    rospy.spin()
    cv2.destroyAllWindows()