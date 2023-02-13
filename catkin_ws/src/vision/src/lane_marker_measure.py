import cv2
import os
import numpy as np
import math

#receives a cv2 image, returns a black and white cv2 image where the "reddest" pixels are black
def thresholdRed(img):
    img_b, img_g, img_r = cv2.split(img) #split by channel
    img_b *= 0 #remove blue color
    img_g *= 0 #remove green color
    tolerance = 0.35
    max_red = np.max(img_r) #get largest value in red color channel
    img_r = np.uint16(img_r) #convert array to uint16 to avoid under/overflow
    img_r -= int(max_red*(1.0-tolerance)) #reduce all values in red color channel by a fraction of the maximum value
            #this makes it so that only the largest values (within the tolerance) in the red channel will still be positive
    np.clip(img_r, 0, 255, out=img_r) #clip all negative values to 0
    img_r = np.uint8(img_r) #make array a uint8 array again (expected by cv2 merge)
    img = cv2.merge((img_b, img_g, img_r)) #merge adjusted channels
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert image to grayscale
    ret,img = cv2.threshold(img,70,255,0) #convert grayscale to black and white with a threshold
    return img

#given an image containing a lane marker, returns one slope per lane marker heading (relative to the image's x/y axis)
#i.e. if lane marker is not fully contained in image, should return one vector
#otherwise should return two vectors
#should also return center point of lane marker (or most central point if not completely contained in image)
def measure_headings(img, debug=False):
    if debug:
        cv2.imshow("original", img)
        cv2.waitKey(0)
    thresh_img = thresholdRed(img)
    if debug:
        cv2.imshow("thresholded/black and white", thresh_img)
        cv2.waitKey(0)
    #get edges of thresholded image (should get the edges of the lane marker)
    edges = cv2.Canny(thresh_img,50,150,apertureSize = 3)
    slopes = []
    #only want up to 4 slopes (one per side of the lane marker)
    while len(slopes) < 4:
        if debug:
            cv2.imshow("remaining edges", edges)
            cv2.waitKey(0)
        #find most prominent line in the image
        line = cv2.HoughLines(edges,1,np.pi/180,10)
        # we use a try statement in case no lines are found
        #when no lines are found a typeerror is thrown
        try:
            for rho,theta in line[0]:
                #compute line coordinates in image using rho and theta
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                #calculate slope from start and end points of line
                slope = (y2-y1)/(x2-x1)
                slopes.append(slope)
                #draw the new line we found on top of the original image
                cv2.line(thresh_img,(x1,y1),(x2,y2),(0,0,255),2)
                #remove the line from the edges image by drawing the line with extra thickness
                #this covers up the line that was detected (edges are in white, the line is drawn in black)
                cv2.line(edges,(x1,y1),(x2,y2),(0,0,0),5)
        except TypeError:
            break
    if debug:
        cv2.imshow("with lines", img)
        cv2.waitKey(0)
    #TODO - return mid point of lane marker as well
    #combine the 4 slopes into 2 most different slopes
    finalSlopes = []
    for i in range(2):
        s1 = min(slopes)
        slopes.remove(s1)
        s2 = min(slopes)
        slopes.remove(s2)
        #average similar slopes to get better approximation of actual slope
        finalSlopes.append((s1+s2)/2)
    return finalSlopes


if __name__ == '__main__':
    #run this script to see the heading detection step by step
    pwd = os.path.realpath(os.path.dirname(__file__))
    test_image_filename = pwd + "\ff.jpg"
    img = cv2.imread(test_image_filename)
    headings = measure_headings(img, debug=True)
    print(headings)
