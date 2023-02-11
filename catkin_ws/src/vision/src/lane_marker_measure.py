import cv2
import os
import numpy as np
import math

def thresholdRed(img):
    img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
    img_b, img_g, img_r = cv2.split(img) #split by channel
    img_b *= 0
    img_g *= 0
    tolerance = 0.35
    max_red = np.max(img_r)
    img_r = np.uint16(img_r)
    img_r -= int(max_red*(1.0-tolerance))
    np.clip(img_r, 0, 255, out=img_r)
    img_r = np.uint8(img_r)
    img = cv2.merge((img_b, img_g, img_r)) #merge adjusted channels
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret,img = cv2.threshold(img,70,255,0)
    return img

#given an image containing a lane marker, returns one vector per lane marker heading (relative to the image's x/y axis)
#i.e. if lane marker is not fully contained in image, will return one vector
#otherwise should return two vectors
def measure_headings(img, debug=False):
    if debug:
        cv2.imshow("out", img)
        cv2.waitKey(0)
    img = thresholdRed(img)
    if debug:
        cv2.imshow("thresholded", img)
        cv2.waitKey(0)
    edges = cv2.Canny(img,50,150,apertureSize = 3)
    slopes = []
    slope1 = 0
    while slope1 != None and len(slopes) < 4:
        if debug:
            cv2.imshow("remaining edges", edges)
            cv2.waitKey(0)
        lines = cv2.HoughLines(edges,1,np.pi/180,10)
        try:
            for rho,theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                slope1 = (y2-y1)/(x2-x1)
                slopes.append(slope1)
                cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
                cv2.line(edges,(x1,y1),(x2,y2),(0,0,255),5)
        except TypeError:
            slope1 = None
    if debug:
        cv2.imshow("thresholded with lines", img)
        cv2.waitKey(0)
    #TODO - reduce 4 slope to average of pairs of most similar slopes
    #TODO - convert two average slopes into heading in degrees using AUV state_theta_z
    return slopes


if __name__ == '__main__':
    
    pwd = os.path.realpath(os.path.dirname(__file__))
    
    test_image_filename = pwd + "\images/frame69_jpg.rf.74f6f59c65c97414344b49e751b95eb2.jpg"
    img = cv2.imread(test_image_filename)
    headings = measure_headings(img, debug=True)
    img = cv2.imread(test_image_filename)
    print(headings)
    line_thickness = 1 # in pixels
    line_x_length = 0.1*img.shape[1] #in pixels, will be 3/4 of bounding box width
    for slope in headings:
        angle = math.degrees(math.atan(slope))
        #on y the slope is inverted because y coordinates grow from the top down in images
        heading_start = (int(max(0.5*img.shape[1] - line_x_length, 0)), int(0.5*img.shape[0] - slope*line_x_length)) # (x,y)
        heading_end = (int(0.5*img.shape[1] + line_x_length), int(max(0.5*img.shape[0] + slope*line_x_length, 0))) # (x,y)
        cv2.line(img, heading_start, heading_end, (0, 0, 255), line_thickness)
        cv2.putText(
            img,
            text=str(-1*angle) + " deg.",
            org=heading_end,
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.4, 
            color=(0, 0, 255), 
            lineType=cv2.LINE_AA,
        )

    cv2.imshow("out", img)
    cv2.waitKey(0)