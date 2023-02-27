import cv2
import os
import numpy as np
import math
from util.k_means import KMeans

def get_colors(img):
    flat_img = np.reshape(img,(-1,3)) / 255
    kmeans = KMeans(n_clusters=10, max_iter=10, init_method='var_part')
    clusters, centers = kmeans.fit(flat_img)
    return np.array(clusters*255)

#return True if basically the same color
#otherwise return False
#assuming color in BGR
def areTheSame(c1, c2, tolerance=0.55):
    for i in (0, 1, 2):
        if min(c1[i], c2[i]) <  max(c1[i], c2[i])*(1.0-tolerance):
            return False
    return True

#receives a cv2 image, returns a black and white cv2 image where the "reddest" pixels are black
def thresholdRed(img):
    colors = get_colors(img)
    img_b, img_g, img_r = colors[:, 0], colors[:, 1], colors[:, 2] #split by channel
    ratio_img = np.uint32(img_r)/(np.uint32(img_g) + np.uint32(img_b) + np.ones(img_b.shape))
    max_i = np.argmax(ratio_img) #get largest value in red color channel
    max_pixel = colors[max_i]
    #max_pixel = img[math.floor(max_i/colors.shape[1])][max_i%colors.shape[1]]

    def mask(x):
        #return black (0,0,0) if similar to max_red within tolerance
        #otherwise return white (255,255,255)
        if areTheSame(x, max_pixel):
            return (0,0,0)
        else:
            return (255,255,255)
    thresh_img = np.uint8(np.zeros(img.shape))
    for r in range(len(img)):
        for p in range(len(img[r])):
            thresh_img[r][p] = mask(img[r][p])
    thresh_img = cv2.blur(thresh_img, (int(0.06*thresh_img.shape[0]),int(0.06*thresh_img.shape[1])))     
    thresh_img = cv2.cvtColor(thresh_img, cv2.COLOR_BGR2GRAY) #convert image to grayscale
    ret,thresh_img = cv2.threshold(thresh_img,70,255,0) #convert grayscale to black and white with a threshold
    return thresh_img

#return the intersection point of two lines of the form (slope, y intercept)
def getIntersection(l1, l2):
    #if m, m' are slopes and b, b' are intercepts the intersection is at x which satisfies:
    #mx+b=m'x+b'
    #mx-m'x=b'-b
    #x=(b'-b)/(m-m')
    if l1[0]-l2[0] == 0:
        return None
    int_x = (l2[1]-l1[1])/(l1[0]-l2[0])
    #y=mx+b
    int_y = int_x*l2[0] + l2[1]
    return (int(int_x), int(int_y))

#given an array of the form ((slope1UpperLine, slope1LowerLine), (slope2UpperLine, slope2LowerLine))
#return the center point of the rectangle defined by the 4 lines
def getCenterPoint(lines, img, debug):
    #get intersection between the two upper lines
    int1 = getIntersection(lines[0][0], lines[1][0])
    #get intersection between the two lower lines
    int2 = getIntersection(lines[0][1], lines[1][1])
    if None in (int1, int2): return None
    #get point halfway between the two intersections
    centerPoint = (int((int1[0]+int2[0])/2), int((int1[1]+int2[1])/2))
    if debug:
        cv2.circle(img, int1, radius=4, color=(255,0,0), thickness=-1)
        cv2.circle(img, int2, radius=4, color=(255,0,0), thickness=-1)
        cv2.circle(img, centerPoint, radius=5, color=(0,0,255), thickness=-1)
        cv2.imshow("with center point", img)
        cv2.waitKey(0)
    return centerPoint

def angleBetweenLines(l1, l2):
    l1_angle = (180*math.atan(l1[0])/math.pi) #between -90 and 90
    l2_angle = (180*math.atan(l2[0])/math.pi) #between -90 and 90
    angle_diff = abs(l1_angle-l2_angle) # between 0 and 180
    if angle_diff > 90:
        return 180.0-angle_diff
    else:
        return angle_diff

#given an image containing a lane marker, returns one slope per lane marker heading (relative to the image's x/y axis)
#i.e should return two slopes
#returns lines in format (l1, l2) where l1 is the heading that is closest to that of the AUV, l2 is heading where the AUV should go
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
    lines = []
    #only want up to 4 slopes (one per side of the lane marker)
    while len(lines) < 4:
        if debug:
            cv2.imshow("remaining edges", edges)
            cv2.waitKey(0)
            pass
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
                x1 = int(x0 + 3000*(-b))
                y1 = int(y0 + 3000*(a))
                x2 = int(x0 - 3000*(-b))
                y2 = int(y0 - 3000*(a))
                #calculate slope from start and end points of line
                if (x2-x1) == 0:
                    slope = 1000000
                else:
                    slope = (y2-y1)/(x2-x1)
                #y = mx+b
                #slope is m, intercept is b
                #therefore the slope is b = y-mx for any point on the line
                intercept = y1-slope*x1
                lines.append((slope, intercept))
                #draw the new line we found on top of the original image
                if debug: 
                    #cv2.line(img,(x1,y1),(x2,y2),(0,0,0),2)
                    pass
                #remove the line from the edges image by drawing the line with extra thickness
                #this covers up the line that was detected (edges are in white, the line is drawn in black)
                cv2.line(edges,(x1,y1),(x2,y2),(0,0,0),10)
        except TypeError:
            break
    if debug:
        #cv2.imshow("with lines", img)
        #cv2.waitKey(0)
        pass
    #combine the 4 lines into 2 most different lines
    finalLines = []
    #array to hold the 4 lines, organized by heading (2 lines per heading, upper and lower)
    laneEdgeLines = []
    if (len(lines) < 4):
        return None, None
    for i in range(2):
        #get slope
        s1 = lines[0]
        lines.remove(s1)
        #get line with least angle from s1
        s2 = min(lines, key=lambda x: angleBetweenLines(s1, x))
        lines.remove(s2)
        #average the lines to get better approximation of actual line
        s1_angle = ((180*math.atan(s1[0])/math.pi) % 180)
        s2_angle = ((180*math.atan(s2[0])/math.pi) % 180)
        #ensure that angles are within 90 degrees of each other so we always average the acute angle between lines
        if abs(s1_angle-s2_angle) > 90:
            s1_angle = s1_angle-180
        avg_angle = (s1_angle+s2_angle)/2
        avg_slope = math.tan(math.pi*avg_angle/180)
        finalLines.append(avg_slope)
        #get the upper and lower lines by comparing their y-intercept values
        upper_line = min((s1,s2), key=lambda x: x[1])
        lower_line = max((s1,s2), key=lambda x: x[1])
        #save upper and lower line to the edge lines array
        laneEdgeLines.append((upper_line,lower_line))
    
    #get the center point of the lane marker using the rectangle defined by the 4 lines on the lane markers edges
    centerPoint = getCenterPoint(laneEdgeLines, img, debug)
    if centerPoint == None: return None, None
    avgs = []
    for slope in finalLines:
        negAvg = getAvgColor(thresh_img, slope, -1, centerPoint)
        posAvg = getAvgColor(thresh_img, slope, 1, centerPoint)
        avgs.append([slope, posAvg, 1])
        avgs.append([slope, negAvg, -1])
    s1 = min(avgs, key=lambda x : x[1])
    avgs.remove(s1)
    s2 = min(avgs, key=lambda x : x[1])
    #negated because y axis is 0 at top of frame
    angle1 = -180*(math.atan(s1[0])/math.pi)
    angle2 = -180*(math.atan(s2[0])/math.pi)
    if s1[2] < 0: #abs(angle) should be above 90
        if abs(angle1) < 90:
            if angle1 < 0:
                angle1 += 180
            else:
                angle1 -= 180
    else: #abs(angle) should be below 90
        if abs(angle1) > 90:
            if angle1 < 0:
                angle1 += 180
            else:
                angle1 -= 180
    if s2[2] < 0: #abs(angle) should be above 90
        if abs(angle2) < 90:
            if angle2 < 0:
                angle2 += 180
            else:
                angle2 -= 180
    else: #abs(angle) should be below 90
        if abs(angle2) > 90:
            if angle2 < 0:
                angle2 += 180
            else:
                angle2 -= 180
    finalLines = [angle1,angle2]
    return finalLines, centerPoint

def getAvgColor(img, slope, direction, center_point):
    x,y = center_point[0], center_point[1]
    step = img.shape[1]/(2*20) #step value
    if slope >1: #big slopes 
        step=step*(1/slope)
    avgColor = 0
    i = 1 
    while (x < img.shape[1] and y < img.shape[0] and x>0 and y>0):
        avgColor += img[y][x]
        x += int(step*direction)
        y += int(slope*step*direction)
        i+=1
    return avgColor/i

def visualizeLaneMarker(img, debug=True):
    #crop image to lane marker
    line_thickness = 1 # in pixels
    line_length = int(0.25*img.shape[1]) #in pixels, line will be 1/4 of bounding box width
    #measure headings from lane marker
    headings, center_point = measure_headings(img, debug)
    if None in (headings, center_point): return img
    for angle in headings:
        #get angle, line start and line end from heading slope
        slope = math.tan((angle/-180)*math.pi)
        #calculate line x length from total length
            #line_length = sqrt(line_x_length^2 + line_y_length^2)
            #line_length^2 = line_x_length^2 + (line_x_length*slope)^2
            #line_length^2 = line_x_length^2 * (1 + slope^2)
            #line_x_length = sqrt(line_length^2 / (1 + slope^2))
        line_x_length = math.sqrt((line_length ** 2) / (1 + slope ** 2))
        if abs(angle) > 90: #heading goes into negative x
            line_end = (int(center_point[0]-line_x_length), int(center_point[1] - slope*line_x_length)) # (x,y)
        else: # heading goes into positive x
            line_end = (int(center_point[0]+line_x_length), int(center_point[1] + slope*line_x_length)) # (x,y)
        #draw line on original image
        cv2.line(img, center_point, line_end, (0,0,255), line_thickness)
        #add text with measured angle of line at the end of the line
        cv2.putText(
            img,
            text=str(angle) + " deg.",
            org=line_end,
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.4, 
            color=(0, 0, 255) , 
            lineType=cv2.LINE_AA,
        )
    cv2.circle(img, center_point, radius=5, color=(0, 0, 255) , thickness=-1)
    return img

if __name__ == '__main__':
    #run this script to see the heading detection step by step
    pwd = os.path.realpath(os.path.dirname(__file__))
    test_image_filename = pwd + "/images/lm (7).png"
    img = cv2.imread(test_image_filename)
    visualizeLaneMarker(img, debug=False)
    cv2.imshow("visualization", img)
    cv2.waitKey(0)
