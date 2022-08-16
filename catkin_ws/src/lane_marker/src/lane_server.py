#!/usr/bin/env python3
import rospy
import numpy as np 
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from lane_marker.srv import LanesInfo, LanesInfoResponse

#TODO #1: Deal with noise or foreign objects
#TODO #2: Deal with case where we don't find lines

BRIGHTNESS_THRESOLD = 50 # Threshold used to compare grayscale image
CANNY_THRESHOLD_1 = 50 # Value for canny edge detection - used for edge linking
CANNY_THRESHOLD_2 = 255 # Used to find initial segments of strong edges
HOUGH_RHO = 6 # Distance resolution of accumulator in pixels
HOUGH_THETA = np.pi / 180 # Angle resolution of accumulator in rad
HOUGH_THRESHOLD = 100 # Accumulator threshold (ie- mumber of votes) - DECREASE FOR MORE LINES
ANGLE_TOLERANCE = 0.4 # Tolerance for angles of considered to belong to the same line 

class Lane_Marker:
    def __init__(self):
        self.bridge = CvBridge() # Create CvBridge object to process ros image

    def image_callback(self,srv):
        ros_img = srv.path_marker_img
        try:
            original = self.bridge.imgmsg_to_cv2(ros_img, 'bgr8') # Convert to opencv form
        except CvBridgeError as e:
            rospy.loginfo('Topic does not exist')
        linesFound = self.findLines(original)
        resultImg, headings, centerX, centerY = self.processLines(linesFound, original.copy())
        return self.sendResponse(headings, centerX, centerY, resultImg)


    def findLines(self, original):
        masked = original.copy() # Create copy of original
        masked[:,:,0:2] = 0 # Keep only red in the image
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY) # Convert to gray(red points will be brighter)
        gray = (gray>BRIGHTNESS_THRESOLD) * gray # Zero out parts  of image that are below thresold
        canny = cv2.Canny(gray, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2) # Perform Canny edge detection on grayscale image to find outlines
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
            thetaFromVertical[rhos<0] = np.pi - thetaFromVertical[rhos<0] # Deal with lines which have negative slope
            fromRef = np.min(thetaFromVertical) # Find point closest to vertical, assuming that we will be nearly aligned with this line
            toRef = np.max(thetaFromVertical) # Find point farthest from the vertical
            linesFrom = houghLines[thetaFromVertical < fromRef+ANGLE_TOLERANCE] # All lines close to vertical
            linesTo = houghLines[thetaFromVertical > toRef-ANGLE_TOLERANCE] # All lines far from vertical
            lineFromAvg = np.average(linesFrom,axis=0) # Take average for both lines
            lineToAvg = np.average(linesTo,axis=0)
            linesAvg = np.array([lineFromAvg,lineToAvg]) # Create array with the average rho and theta values
            return linesAvg
    
    def processLines(self, linesAvg, resultImg):
        color = (0,0,255)  # Set color to red initially, to represent 'from' lane
        rho = linesAvg[:,0,0]
        theta = linesAvg[:,0,1]
        a = np.cos(theta)
        b = np.sin(theta)
        # Taking x and y components of ax + by = c, where c=rho
        eqns = np.column_stack((a,b))
        centerX, centerY = np.linalg.solve(eqns,rho)
        x0 = a * rho # rho = x.cos(theta) + y.sin(theta)
        y0 = b * rho
        #Lines for plotting taken from Opencv houghlines docs
        pt1 = (int(x0[0] + 1000*(-b[0])), int(y0[0] + 1000*(a[0])))
        pt2 = (int(x0[0] - 1000*(-b[0])), int(y0[0] - 1000*(a[0])))
        cv2.line(resultImg, pt1, pt2, color, 2, cv2.LINE_AA) # Plot line
        color = (0,255,0) # Change color to  green for 'to' line
        pt3 = (int(x0[1] + 1000*(-b[1])), int(y0[1] + 1000*(a[1])))
        pt4 = (int(x0[1] - 1000*(-b[1])), int(y0[1] - 1000*(a[1])))
        cv2.line(resultImg, pt3, pt4, color, 2, cv2.LINE_AA) # Plot line 
        cv2.circle(resultImg,(int(centerX),int(centerY)),10,color=(255,0,150),thickness=3) # Draw a circle at the center
        headings = theta # Initial headings
        headings[rho>0] = -headings[rho>0] # Clockwise headings are negative
        headings[rho<0] = np.pi - headings[rho<0] # Conversion for certain slopes
        headingsInDegrees = headings * 180/np.pi # Convert headings to degrees
        print('Heading from: ', headingsInDegrees[0])
        print('Heading to: ', headingsInDegrees[1])
        print('Center at: ', centerX, ', ', centerY)
        return resultImg, headingsInDegrees, centerX, centerY
    
    def sendResponse(self, headings, centerX, centerY, resultImg):
        rosResponseImg = self.bridge.cv2_to_imgmsg(resultImg,'bgr8')
        center = Point()
        headingFrom = headings[0]
        headingTo = headings[1]
        center.x = centerX
        center.y = centerY
        server_response = LanesInfoResponse(rosResponseImg, headingTo, headingFrom, center)
        return server_response


if __name__=='__main__':
    rospy.init_node('lane_marker',anonymous=True)
    lane_marker = Lane_Marker()
    serv = rospy.Service('lanes_info',LanesInfo,lane_marker.image_callback)
    rospy.spin()