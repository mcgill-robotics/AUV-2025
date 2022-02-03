#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from std_msgs.msg import Float64
from cv.msg import CvTarget
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from dynamic_reconfigure.server import Server
from cv.cfg import laneDetectorParamsConfig

''' 
    Currently, there are two methods in this script, with the intention that 
    both get tested and we decide on one and archive the other. Both 
    methods do some image pre-processing, that looks like:
	1) Gaussian blur
	2) Increase red channel
	3) Mask by color to produce a binary image
	
	At this point, the methods diverge. The old method works by
	4a) Find contours in this mask.
	5a) Check if they are large enough to be the lane
	6a) If they are, run canny edge detection to find the edges
	7a) Average the angles of the Edges to find the slopes
	
	The new method works by
	4b) Finding the moments
	5b) using the moments to return the centroid
	6b) Do a line fit to determine the angle of the lane
	
    I strongly suspect the new method with supercede the old;
    I think it is likely to be more robust. Testing will tell.
'''
class LaneDetector():

    def __init__(self):
        self.pubTargetLines       = rospy.Publisher('cv/down_cam_target_lines'    , CvTarget, queue_size=1)
        self.pubTargetCentroid    = rospy.Publisher('cv/down_cam_target_centroid' , CvTarget, queue_size=1)
        self.pubHeadingFit        = rospy.Publisher('cv/down_cam_heading_Fitting' , Float64 , queue_size=1)
        self.pubHeadingCentroid   = rospy.Publisher('cv/down_cam_heading_Centroid', Float64 , queue_size=1)
        self.pubHeadingHoughLines = rospy.Publisher('cv/down_cam_heading_Hough'   , Float64 , queue_size=1) 
        
        #The following are two publishers that output Float64s to feed directly into the PIDs. 
        #I'm having trouble getting the CvTarget data to play nice...which seems very odd, but w/e.
        self.pubPIDx              = rospy.Publisher('cv/down_cam_PIDx', Float64 , queue_size=1)
        self.pubPIDy              = rospy.Publisher('cv/down_cam_PIDy', Float64 , queue_size=1)  
            
        self.bridge               = CvBridge()
        self.sub                  = rospy.Subscriber("/image_raw", Image, self.callback)
        self.angle_top_lane       = None
        self.laneFound            = False
        self.smoothQueue          = deque([])
        self.debugimgs            = True #When true, show debugging images
        print("starting laneDetector")

    def getAngleOfTopLine(self, points, difSlopes, img):
        '''
        Find the line with higher YCoordinate
        '''
        maxY   = np.inf
        maxIdx = -1
        for idx, (x, y) in enumerate(points):
            if y < maxY:
                maxIdx = idx
                maxY   = y
        # debug
        #cv2.circle(img, points[maxIdx], 25, (0, 255, 255), -1)

        angle = math.degrees(math.atan(difSlopes[maxIdx]))

        # angle = math.atan(difSlopes[maxIdx])
        if angle >= 0:
            angle = 90 - angle
        if angle < 0:
            angle = -(90 + angle)

        self.angle_top_lane = math.radians(angle)

        print("Turn by {} degrees! = {} rad".format(angle,self.angle_top_lane))


    def callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.display_debug_image(self.debugimgs,img,'Original',0.5,-6000,6000)
        xReturn = None
        yReturn = None

        #blur
        img_blur = cv2.GaussianBlur(img, 
        	           (self.blurKernelSize, self.blurKernelSize), 
        	            self.blurSigma)
        self.display_debug_image(self.debugimgs,img_blur,'Blurred',0.5,-6000,6000)

        # increase red
        img[:, :, 2] = np.multiply(img_blur[:, :, 2], self.redChannelXFactor)
        img_redbump  = np.clip(img_blur, 0, 255)
        self.display_debug_image(self.debugimgs,img_redbump,'Red Channel Increased',0.5,-6000,6000)

        #Convert to HVT to facilitate colormasking
        img_hvt          = cv2.cvtColor(img_redbump, cv2.COLOR_BGR2HSV)

    	#define lower and upper colormask bounds
        lower_cm_bound = np.array([self.hueLowerBound,
        	                       self.satLowerBound,
        	                       self.valLowerBound])
        upper_cm_bound = np.array([self.hueUpperBound,
        	                       self.satUpperBound,
        	                       self.valUpperBound])

        #Generate and display an image that indicates the masking bounds
        img_bounds       = self.generate_mask_color_image(img_hvt,lower_cm_bound,upper_cm_bound)
        self.display_debug_image(self.debugimgs,img_bounds,'Colormask bounds',0.5,-6000,6000)

        # Filter by Color to obtain a colormask
        MIN_CONTOUR_SIZE = rospy.get_param("/cv/lanes/orange_cnt_size", 10000)
        mask             = cv2.inRange(img_hvt, lower_cm_bound, upper_cm_bound)
        self.display_debug_image(self.debugimgs,mask,'Colormask Output',0.5,-6000,6000)

        
        #Using the colormask, find contours
        #This line does heavy lifting with opencv
       
        im2, cnt, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)


		######################################################################
        #This is where the methods diverge. The Next bit is 
        #the 'old' method uses hough lines to find the lines on an 
        #Edge detected image. It appears to me that this method it
        #Extremely noisy with the current parameters, but perhaps 
        #With tweaking it could be improved.  
        ######################################################################

        if len(cnt) == 0:
            print("No lane of sufficient size found")
            self.laneFound      = False
            self.angle_top_lane = None
            #return

        else:
            #Contours were found
            # find contour with biggest area
            biggestCont = max(cnt, key=cv2.contourArea)
            # check contour size

            if (cv2.contourArea(biggestCont) < MIN_CONTOUR_SIZE):
                print("No lane of sufficient size found")
                print(cv2.contourArea(biggestCont), MIN_CONTOUR_SIZE)
                self.laneFound      = False
                self.angle_top_lane = None
                #return
            else:
                # draw in blue the contours that were found
                cv2.drawContours(img, [biggestCont], -1, (255, 255, 0), 3, 8)
                # create emptyImage for mask
                contourMask = np.zeros(img.shape, np.uint8)
                cv2.drawContours(contourMask, [biggestCont], -1, (255, 255, 255), cv2.FILLED, 8)
                

                edges = cv2.Canny(contourMask,
                                  self.cannyLowerThreshold,
                                  self.cannyUpperThreshold)
                self.display_debug_image(self.debugimgs,edges,'Canny Edges',0.5,-6000,6000)


                # TODO: check if these values work reliably
                # Hough Lines Parameter update
                rho             = self.houghRho            # distance resolution in pixels of the Hough grid
                theta           = self.houghTheta          # angular resolution in radians of the Hough grid
                threshold       = self.houghThreshold      # minimum number of votes (intersections in Hough grid cell)
                min_line_length = self.houghMinLineLength  # minimum number of pixels making up a line
                max_line_gap    = self.houghMaxLineGap     # maximum gap in pixels between connectable line segments
                line_image      = np.copy(img) * 0         # creating a blank to draw lines on
                # Run Hough on edge detected image
                # Output "lines" is an array containing endpoints of detected line segments
                lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                        min_line_length, max_line_gap)
                print("Found Lines: {}".format(len(lines)))

                # draw Lines
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        cv2.line(img, (x1, y1), (x2, y2), (100, 230, 230), 5)

                # average all lines with similar direction together
                slopes = []
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        slope = 1.0 * (y2 - y1) / (x2 - x1)
                        # to avoid infinite slope
                        if slope == float("Inf") or slope > 25:
                            slope = 25
                        if slope == -float("Inf") or slope < -25:
                            slope = 25

                    slopes.append(slope)

                # stores the line Coords
                lineGroups = [[lines[0]]]
                # stores the slopes of all
                slopeCollection = [[slopes[0]]]
                difSlopes = []
                difSlopes.append(slopes[0])

                #group lines with similar slope together
                for no, slope in enumerate(slopes[1:], start=1):
                    similarFound = False
                    for idx, pos in enumerate(difSlopes):
                        # TODO: possibly adapt values
                        lowerBound = pos - (0.4 * abs(pos)) - 0.1
                        upperBound = pos + (0.4 * abs(pos)) + 0.1

                        # case where slope is similar
                        if slope >= lowerBound and slope <= upperBound:
                            slopeCollection[idx].append(slope)
                            lineGroups[idx].append(lines[no])
                            similarFound = True
                            # set the slope of the group to the mean
                            difSlopes[idx] = np.mean(slopeCollection[idx], 0)
                            break
                    if similarFound == False:
                        difSlopes.append(slope)
                        slopeCollection.append([slope])
                        lineGroups.append([lines[no]])

                #Debugging, TC
                #print("Number of Found Slopes: {}".format(len(difSlopes)))

                # find the mean point of lineGroup to calculate intersection
                # sort LineGroup by size to get the groups with most entries
                relevantLines = zip(lineGroups, difSlopes)
                relevantLines.sort(key=lambda x: len(x[0]), reverse=True)
                lineGroups, difSlopes = zip(*relevantLines)

                #debug
                #for l in lineGroups:
                    #print(len(l))
                #print(difSlopes)

                xReturn = None
                yReturn = None

                centerPoints = []
                intercepts = []


                if (len(lineGroups) == 1):
                    # there is only one slope found
                    lg = lineGroups[0]
                    #print("NOT 2 LINEGROUPS, calculating center")
                    xPoint = int((np.mean(lg, 0)[0, 0] + np.mean(lg, 0)[0, 2]) / 2)
                    yPoint = int((np.mean(lg, 0)[0, 1] + np.mean(lg, 0)[0, 3]) / 2)
                    #cv2.circle(img, (xPoint, yPoint), 30, (255, 0, 0), -1)
                    xReturn = xPoint
                    yReturn = yPoint


                elif(len(lineGroups) >= 2):
                    # only look at the 2 biggest groups (ignores irrelevant lines)
                    for i, lg in enumerate(lineGroups[:2]):
                        xPoint = int((np.mean(lg, 0)[0, 0] + np.mean(lg, 0)[0, 2]) / 2)
                        yPoint = int((np.mean(lg, 0)[0, 1] + np.mean(lg, 0)[0, 3]) / 2)

                        intercept = yPoint - (xPoint * difSlopes[i])
                        intercepts.append(intercept)
                        #debug
                        #cv2.circle(img, (xPoint, yPoint), 10, (0, 255, 0), -1)
                        centerPoints.append((xPoint, yPoint))

                    #only look at the 2 most occuring slopes
                    difSlopes = difSlopes[:2]

                    # calculate Intersection Point
                    intersectionX = int((intercepts[1] - intercepts[0]) / (difSlopes[0] - difSlopes[1]))
                    intersectionY = int(
                        (difSlopes[0] * intercepts[1] - difSlopes[1] * intercepts[0]) / (difSlopes[0] - difSlopes[1]))

                    #calculate the angle between lanes to make sure a correct lane was found
                    angle = math.degrees(math.atan((difSlopes[0] - difSlopes[1]) / (1 + difSlopes[0] * difSlopes[1])))
                    #print("Angle = {}".format(angle))

                    if abs(angle) > 30:
                        # this is the normal case of 2 lane parts found
                        #cv2.circle(img, (intersectionX, intersectionY), 30, (0, 255, 0), -1)
                        xReturn = intersectionX
                        yReturn = intersectionY

                        self.getAngleOfTopLine(centerPoints, difSlopes, img)

                    else:
                        # 2 different slopes found that don't have big angle
                        # finds 2 slopes even tho it should be just one
                        xMean = 0
                        yMean = 0
                        for lg in lineGroups[:2]:
                            xMean += (np.mean(lg, 0)[0, 0] + np.mean(lg, 0)[0, 2]) / 2
                            yMean += (np.mean(lg, 0)[0, 1] + np.mean(lg, 0)[0, 3]) / 2
                        xMean = int(xMean / 2.0)
                        yMean = int(yMean / 2.0)

                        #cv2.circle(img, (xMean, yMean), 30, (255, 0, 255), -1)
                        xReturn = xMean
                        yReturn = yMean
            if (xReturn != None):
                #new
                xReturn,yReturn = self.smoothPoint(xReturn,yReturn)

                cv2.circle(img, (int(xReturn), int(yReturn)), 10, (255, 0, 255), -1)

                msg = CvTarget()
                msg.gravity.x = xReturn
                msg.gravity.y = yReturn
                msg.gravity.z = 0
                msg.probability.data = 1.0
                self.pubTargetLines.publish(msg)
                self.laneFound = True
            else:
                #If no correct lane is found we want to make sure to turn the servo off
                #  print("no lane") --------------------------------------------------------------------------
                msg = CvTarget()
                #This makes sure the servo turns off
                msg.probability.data = 0.0
                self.pubTargetLines.publish(msg)
                #also clear the queue
                self.smoothQueue = deque([])
                self.laneFound = False

            small = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow("Final Image", small)
            cv2.waitKey(5)

		######################################################################
        #This is where the new methods begins
        #To get the center, use the centroid formula 
        #To get the angle, either use fancy centroid formulas, or rely on
        #OpenCV line fitting algorithms!
        ######################################################################    
        #First, find the convex hull
        hull     = cv2.convexHull(biggestCont)  
        hull_img = np.zeros(img.shape, np.uint8)
        cv2.drawContours(hull_img ,[hull],0, (255, 255, 255),cv2.FILLED, 8)
		
		# convert image to grayscale image
        gray_img   = cv2.cvtColor(hull_img, cv2.COLOR_BGR2GRAY)
        # convert the grayscale image to binary image
        ret,thresh = cv2.threshold(gray_img,127,255,cv2.THRESH_BINARY)
        #The above two steps are necessary because cv2.moments needs 
        #A binary, single channel, input!

        #Now, with this binary convex hull, compute the moments! 
        M = cv2.moments(thresh) 

        #The centroid can be found using these moments as
        xCentroid = M["m10"]/M["m00"]
        yCentroid = M["m01"]/M["m00"]
        #Let's slap this centroid on the image and display it 
        cv2.circle(hull_img, (int(xCentroid), int(yCentroid)), 10, (255, 0, 255), -1)

        #Okay, now shit is going to get a bit wild. 
        #We need to use some hard math to get the orientation. Check this link out
        #http://raphael.candelier.fr/?blog=Image%20Moments
        #I'm going to start out with an intermediary step to make life easier 
        a = M["m20"]/M["m00"]    - xCentroid**2;
        b = 2*(M["m11"]/M["m00"] - xCentroid*yCentroid);
        c = M["m02"]/M["m00"]    - xCentroid**2;
        d = b/(a-c)

        #With these wacky bois in hand, we can find the orientation
        thetaCentroid = (1.0/2.0)*np.arctan(d)

        #alternatively, we could be less smart and just use CVs line fitting
        img, cnts ,heir  = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnt              = cnts[0]
        rows,cols        = hull_img.shape[:2]
        [vx,vy,x,y]      = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
        lefty            = int((-x*vy/vx) + y)
        righty           = int(((cols-x)*vy/vx)+y)
        hull_img         = cv2.line(hull_img,(cols-1,righty),(0,lefty),(0,255,0),2)
        self.display_debug_image(self.debugimgs,hull_img,'Convex Hull',0.5,-6000,6000)

        #let's spit everything to a topic so we can do some plotting 
        #### First, the CV Targets ########################################

        msgCentroid                  = CvTarget()
        msgCentroid.gravity.x        = xCentroid
        msgCentroid.gravity.y        = yCentroid
        msgCentroid.gravity.z        = 0
        msgCentroid.probability.data = 1.0
          
        
        ## (The other one is published above in the code where it is generated)
        #### Next, the headings ########################################
        self.pubHeadingCentroid.publish(thetaCentroid) 
        self.pubHeadingFit.publish(np.arctan(vy/vx)) 

        #Only publish something to the Hough Lines topic if the hough lines algorithm has actually been run!
        if self.angle_top_lane != None:
            print('Publishing to the Hough Lines topic!')
            self.pubHeadingHoughLines.publish(self.angle_top_lane) 
            # Also publish the centroid! 
            self.pubTargetCentroid.publish(msgCentroid)   
            #self.pubPIDx.publish(xCentroid)
            #self.pubPIDy.publish(yCentroid) 

        

    def getAngle(self):
        return self.angle_top_lane

    def getLaneFound(self):
        return self.laneFound

    def stop(self):
        self.sub.unregister()

    def smoothPoint(self,xVal,yVal):
        if len(self.smoothQueue) < self.smoothQueueSize:
            self.smoothQueue.append((xVal,yVal))
        else:
            self.smoothQueue.popleft()
            self.smoothQueue.append((xVal, yVal))

        meanX = 0.0
        meanY = 0.0
        for x,y in self.smoothQueue:
            meanX += x
            meanY += y

        meanX = meanX / len(self.smoothQueue)
        meanY = meanY / len(self.smoothQueue)

        return meanX, meanY

    def dyn_reconf_cb(self, config, level):
    	'''
    	%This is the dynamic reconfigure callback.
    	This function updates all of the
    	reconfigurable parameters. It is fed into the Server
    	and updates everything defined in the .cfg file
    	'''

    	#Debugging
    	#rospy.loginfo('{}'.format(config))

    	#These are values for color masking
    	self.hueLowerBound       = config.hueLowerBound
    	self.hueUpperBound       = config.hueUpperBound
    	self.satLowerBound       = config.satLowerBound
    	self.satUpperBound       = config.satUpperBound
        self.valLowerBound       = config.valLowerBound
        self.valUpperBound       = config.valUpperBound
        #These values are for the canny edge detection
        self.cannyLowerThreshold = config.cannyLowerThreshold
        self.cannyUpperThreshold = config.cannyUpperThreshold
        #These values are for the hough line detection
        self.houghRho            = config.houghRho 
        self.houghTheta          = config.houghTheta
        self.houghThreshold      = config.houghThreshold
        self.houghMinLineLength  = config.houghMinLineLength
        self.houghMaxLineGap     = config.houghMaxLineGap
        #These are values for gaussian blurring
        self.blurKernelSize      = config.blurKernelSize
        self.blurSigma           = config.blurSigma
        #This is for the smoothing of the output target
        self.smoothQueueSize     = config.smoothQueueSize
 		#This is for artificially inflating The red channel
        self.redChannelXFactor   = config.redChannelXFactor

    	return config

    def generate_mask_color_image(self,img,lower_cm_bound,upper_cm_bound):
    	###################################################
    	#This function makes a dummy image that sets half
    	#of the image to the lower bound color, and
    	#the other half to the upper bound color
    	###################################################
        row,col,plane = img.shape
        boundimg      = np.zeros((row,col,plane),np.uint8)

        #lower bounds
        boundimg[:,0:col/2,0] = lower_cm_bound[0]
        boundimg[:,0:col/2,1] = lower_cm_bound[1]
        boundimg[:,0:col/2,2] = lower_cm_bound[2]

        #upper bounds
        boundimg[:,col/2:col,0]= upper_cm_bound[0]
        boundimg[:,col/2:col,1]= upper_cm_bound[1]
        boundimg[:,col/2:col,2]= upper_cm_bound[2]

        #Convert HSV values back to BGR so we can display them
        boundimg = cv2.cvtColor(boundimg, cv2.COLOR_HSV2BGR)
        return boundimg

    def display_debug_image(self,debug,img,label,scaling,dispx,dispy):
    	'''
    	This function displays the input image.
    	I call it a lot for debugging, so the function
    	is convenient.
    	Only does anything if debug == True
    	#TODO: make window scaling less crappy
    	'''
    	if (debug == True):
    		small = cv2.resize(img, (0, 0), fx=scaling, fy=scaling)
    		cv2.imshow(label, small)
        	#cv2.moveWindow(label, dispx,dispy);
        	cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('LaneDetector',anonymous=True)
    bla = LaneDetector()
    srv = Server(laneDetectorParamsConfig, bla.dyn_reconf_cb)
    rospy.spin()
