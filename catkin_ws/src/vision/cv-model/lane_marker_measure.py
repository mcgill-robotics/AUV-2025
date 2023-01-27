import cv2
import os
import numpy as np


def thresholdRed(img):
    img = cv2.resize(img, (0,0), fx=0.25, fy=0.25) 
    img_b, img_g, img_r = cv2.split(img) #split by channel
    img_b *= 0
    img_g *= 0
    tolerance = 0.4
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
def measure_headings(img):
    img = thresholdRed(img)
    cv2.imshow("out", img)
    cv2.waitKey(0)
    edges = cv2.Canny(img,50,150,apertureSize = 3)
    cv2.imshow("out", edges)
    lines = cv2.HoughLines(edges,5,np.pi/180,10)
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
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    except TypeError:
        slope1 = None
    lines = cv2.HoughLines(edges,2,np.pi/180,10)
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
            slope2 = (y2-y1)/(x2-x1)
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    except TypeError:
        slope2 = None
    cv2.imshow("out", img)
    cv2.waitKey(0)
    return (slope1, slope2)


if __name__ == '__main__':
    
    pwd = os.path.realpath(os.path.dirname(__file__))
    
    test_image_filename = pwd + "/data/raw/images/frame44_jpg.rf.36a74eb74ab5692f83b66d8ff2cb12c6.jpg"
    test_image = cv2.imread(test_image_filename)
    print(measure_headings(test_image))
