from cv2 import line
import numpy as np 
import cv2
import math

#TODO #1: Calculate angles to be published
#TODO #2: Incorporate into ros
#TODO #3: Deal with noise or foreign objects
#TODO #4: Deal with case where we don't find lines

BRIGHTNESS_THRESOLD = 50 # Threshold used to compare grayscale image
CANNY_THRESHOLD_1 = 50 # Value for canny edge detection - used for edge linking
CANNY_THRESHOLD_2 = 255 # Used to find initial segments of strong edges
HOUGH_RHO = 6 # Distance resolution of accumulator in pixels
HOUGH_THETA = np.pi/180 # Angle resolution of accumulator in rad
HOUGH_THRESHOLD = 100 # Accumulator threshold (ie- mumber of votes) - DECREASE FOR MORE LINES

original = cv2.imread('source.png') # Initially using static image
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
    linesFrom = houghLines[thetaFromVertical<fromRef+0.4] # All lines close to vertical
    linesTo = houghLines[thetaFromVertical>toRef-0.4] # All lines far from vertical
    lineFromAvg = np.average(linesFrom,axis=0) # Take average for both lines
    lineToAvg = np.average(linesTo,axis=0)
    linesAvg = np.array([lineFromAvg,lineToAvg]) # Create array with the average rho and theta values
    lines = linesAvg.tolist() # Convert to list for plotting

grads = [] # List of gradients
yints = [] # List of y-intercepts
color = (0,0,255)  # Set color to red initially, to represent 'from' lane

for i in range(len(lines)):
    rho = lines[i][0][0]
    theta = lines[i][0][1]
    a = math.cos(theta)
    b = math.sin(theta)
    x0 = a * rho # rho = x.cos(theta) + y.sin(theta)
    y0 = b * rho
    grad = -a/b # convert to y = mx+c OR y = [rho-x.cos(theta)]/sin(theta). Gradient is -cos/sin = -cot(theta)
    grads.append(grad) # Append current gradient to list
    #Lines for plotting taken from Opencv houghlines docs
    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    yint = rho / b # y = [rho-x.cos(theta)]/sin(theta), so the y-intercept or c is rho/sin(theta)
    yints.append(yint) # Append to list of y-intercepts
    cv2.line(hough, pt1, pt2, color, 2, cv2.LINE_AA) # Plot line
    color = (0,255,0) # Change color to  green for 'to' line

centerX = (yints[1]-yints[0])/(grads[0]-grads[1]) # Caculate the intersection of the lines, where m1x + c1 = m2X + c2
centerY = centerX * grads[0] + yints[0] # y = mx + c
cv2.circle(hough,(int(centerX),int(centerY)),10,color=(255,0,150),thickness=3) # Draw a circle at the center


# Only display
cv2.imshow('Original',original)
#cv2.imshow('Masked',masked)
#cv2.imshow('Gray',gray)
cv2.imshow('Canny',canny)
cv2.imshow('Hough',hough)
cv2.waitKey(0)
cv2.destroyAllWindows()