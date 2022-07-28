#!/usr/bin/env python3
import cv2
import numpy as np
from cv_bridge import CvBridge
import rospy
from quali_gate.srv import GateInfo, GateInfoResponse
from geometry_msgs.msg import Point

class Gate_CV:
    def __init__(self):
        self.bridge = CvBridge()

    def imageCallback(self, srv):
        '''
        Callback for service call
        '''
        ros_image = srv.gate_image
        image = self.bridge.imgmsg_to_cv2(ros_image)
        frameCenX, frameCenY, lines = self.getLines(image)
        gateCenX, gateCenY, linesAvg = self.processLines(lines)
        print('Center of gate is at: ', gateCenX, ',', gateCenY)
        print('Center of frame is at ', frameCenX, ',', frameCenY)
        resImg = self.finalImg(image, gateCenX, gateCenY, frameCenX, frameCenY, linesAvg)
        rosResImg = self.bridge.cv2_to_imgmsg(resImg, 'bgr8')
        gateCenter = Point()
        frameCenter = Point()
        gateCenter.x = gateCenX
        gateCenter.y = gateCenY
        frameCenter.x = frameCenX
        frameCenter.y = frameCenY
        return GateInfoResponse(rosResImg, frameCenter, gateCenter)


    def getLines(self, image):
        '''
        Returns the lines from houghlines algorithm
        '''
        redMasked = image.copy()
        redMasked[:,:,0:2] = 0
        gray = cv2.cvtColor(redMasked, cv2.COLOR_BGR2GRAY)
        gray = gray * (gray>50)
        canny = cv2.Canny(gray, 50, 150)

        lines = cv2.HoughLinesP(canny, rho=6, theta=np.pi/180, threshold=60, minLineLength=200, maxLineGap=70)
        frameCenterX = np.size(canny, 1) / 2
        frameCenterY = np.size(canny, 0) / 2
        
        return frameCenterX, frameCenterY, lines

    def processLines(self, lines):
        if lines is not None:
            linesnp = np.array(lines[:,0,:]) # Convert to np array
            delx = np.abs(linesnp[:,0]-linesnp[:,2])
            dely = np.abs(linesnp[:,1]-linesnp[:,3])
            horizontal = linesnp[dely<50]
            vertical = linesnp[delx<50]
            leftRef = np.min(vertical[:,0], axis=0)
            left = vertical[vertical[:,0] < leftRef+50]
            rightRef = np.max(vertical[:,0],axis=0)
            right = vertical[vertical[:,0] > rightRef-50]
            topAvg = np.average(horizontal, axis=0)
            leftAvg = np.average(left, axis=0)
            rightAvg = np.average(right, axis=0)
            linesAvg = np.array([topAvg, leftAvg, rightAvg]).tolist()
            bottomY = (leftAvg[1]+rightAvg[1]) / 2 # Take bottom of verticals
            topY = (topAvg[1]+topAvg[3]) / 2
            avgY = (bottomY+topY) / 2
            avgX = (leftAvg[0]+leftAvg[2]+rightAvg[0]+rightAvg[2]) / 4
            return avgX, avgY, linesAvg

    def finalImg(self, img, centerX, centerY, frameCenterX, frameCenterY, lines):
        resultImg = img.copy()
        for i in range(0, len(lines)):
            l = lines[i]
            cv2.line(resultImg, (int(l[0]), int(l[1])), (int(l[2]), int(l[3])), (0,150,0), 3, cv2.LINE_AA)
        cv2.drawMarker(resultImg, (int(frameCenterX), int(frameCenterY)), color=( 0,0,255), markerType=cv2.MARKER_TILTED_CROSS,
        thickness=2, markerSize=10)
        cv2.circle(resultImg, (int(centerX), int(centerY)), 5, (0,255,0), -1) # Draw circle at frame center
        return resultImg

if __name__=='__main__':
    gate_cv = Gate_CV()
    rospy.init_node('listener_node',anonymous=True)
    imgServer = rospy.Service('gate_info', GateInfo, gate_cv.imageCallback)
    rospy.spin()