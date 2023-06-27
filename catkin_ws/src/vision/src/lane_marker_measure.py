import cv2
import os
import numpy as np
import math
from cv_bridge import CvBridge

longest_downscaled_size = None
blur1_amt = None
color_tolerance = None
blur2_amt = None
bridge = CvBridge()

#return True if basically the same color
#otherwise return False
#assuming color in BGR
def areTheSame(c1, c2):
    #make color brightnesses the same (just care about color ratios, not brightness/saturation) by making max value 255 in any channel
    c1 += min((255,255,255)-c1)
    c2 += min((255,255,255)-c2)
    for i in (0, 1, 2):
        if max(c1[i], c2[i]) - min(c1[i], c2[i]) > color_tolerance*255:
            return False
    return True

#receives a cv2 image, returns a black and white cv2 image where the "reddest" pixels are black
def thresholdRed(img, downscale_publisher=None, blur1_publisher=None, tol_publisher=None, blur2_publisher=None, thresh_publisher=None):
    getParameters()
    if min(img.shape[0], img.shape[1]) > longest_downscaled_size:
        scaling_factor = longest_downscaled_size/min(img.shape[0], img.shape[1])
        downscaled_size = (int(img.shape[1]*scaling_factor), int(img.shape[0]*scaling_factor))
    else:
        downscaled_size = (img.shape[1], img.shape[0])
    downscaled = cv2.resize(img, dsize=downscaled_size, interpolation=cv2.INTER_AREA)

    if blur1_amt > 0: blurred = cv2.blur(downscaled, (int(blur1_amt*downscaled.shape[0]),int(blur1_amt*downscaled.shape[1])))
    else: blurred = downscaled
    img_b, img_g, img_r = cv2.split(blurred)
    #normalize colors so that the max color channel in every pixel is 255
    max_brightness_increase = np.min(255-blurred, axis=2)
    ratio_img = np.uint32(img_r+max_brightness_increase)/(np.uint32(img_g+max_brightness_increase) + np.uint32(img_b+max_brightness_increase) + np.uint32(img_r+max_brightness_increase))
    max_i = np.argmax(ratio_img) #get largest value in red color channel
    max_pixel = blurred[math.floor(max_i/blurred.shape[1])][max_i%blurred.shape[1]]
    if downscale_publisher != None: downscale_publisher.publish(bridge.cv2_to_imgmsg(downscaled, "bgr8")) #FOR ADJUSTING VALUES
    if blur1_publisher != None: blur1_publisher.publish(bridge.cv2_to_imgmsg(blurred, "bgr8")) #FOR ADJUSTING VALUES
    
    def mask(x):
        #return black (0,0,0) if similar to max_red within tolerance
        #otherwise return white (255,255,255)
        if areTheSame(x, max_pixel):
            return (0,0,0)
        else:
            return (255,255,255)

    thresh_img = np.uint8(np.zeros(blurred.shape))
    for r in range(len(blurred)):
        for p in range(len(blurred[r])):
            thresh_img[r][p] = mask(blurred[r][p])

    if tol_publisher != None: tol_publisher.publish(bridge.cv2_to_imgmsg(thresh_img, "bgr8")) #FOR ADJUSTING VALUES

    thresh_img = cv2.resize(thresh_img, dsize=(int(img.shape[1]), int(img.shape[0])), interpolation=cv2.INTER_AREA)
    thresh_img = cv2.blur(thresh_img, (int(blur2_amt*thresh_img.shape[0]),int(blur2_amt*thresh_img.shape[1])))     
    
    if blur2_publisher != None: blur2_publisher.publish(bridge.cv2_to_imgmsg(thresh_img, "bgr8")) #FOR ADJUSTING VALUES
    
    thresh_img = cv2.cvtColor(thresh_img, cv2.COLOR_BGR2GRAY) #convert image to grayscale
    ret,thresh_img = cv2.threshold(thresh_img,70,255,0) #convert grayscale to black and white with a threshold
    
    if thresh_publisher != None: thresh_publisher.publish(bridge.cv2_to_imgmsg(thresh_img, "mono8")) #FOR ADJUSTING VALUES
    
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
def getCenterPoint(lines):
    #get intersection between the two upper lines
    int1 = getIntersection(lines[0][0], lines[1][0])
    #get intersection between the two lower lines
    int2 = getIntersection(lines[0][1], lines[1][1])
    if None in (int1, int2): return None
    #get point halfway between the two intersections
    centerPoint = (int((int1[0]+int2[0])/2), int((int1[1]+int2[1])/2))
    return centerPoint

def angleBetweenLines(l1, l2):
    l1_angle = (180*math.atan(l1[0])/math.pi) #between -90 and 90
    l2_angle = (180*math.atan(l2[0])/math.pi) #between -90 and 90
    angle_diff = abs(l1_angle-l2_angle) # between 0 and 180
    if angle_diff > 90:
        return 180.0-angle_diff
    else:
        return angle_diff

def getParameters():
    global longest_downscaled_size
    global blur1_amt
    global color_tolerance
    global blur2_amt
    # Read the values for downscaling, bluring and color tolerance from a text file
    values = []
    with open(pwd + '/camera_calibrations/thresholding_values.txt', 'r') as file:
        # Read the file line by line
        for line in file:
            # Ignore lines that start with "#"
            if line[0] != '#' :
                # Extract the values from the line
                value = line.strip()
                if value != '\n' and value != '':
                    values.append(float(value))
    longest_downscaled_size = values[0]
    blur1_amt = values[1]
    color_tolerance = values[2]
    blur2_amt = values[3]


#given an image containing a lane marker, returns one slope per lane marker heading (relative to the image's x/y axis)
#i.e should return two slopes
#returns lines in format (l1, l2) where l1 is the heading that is closest to that of the AUV, l2 is heading where the AUV should go
#should also return center point of lane marker (or most central point if not completely contained in image)
def measure_headings(img, debug=False, debug_img=None):
    if debug_img is None: debug_img = img
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
        line = cv2.HoughLines(edges,1,np.pi/180,25)
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
                #remove the line from the edges image by drawing the line with extra thickness
                #this covers up the line that was detected (edges are in white, the line is drawn in black)
                line_thickness = max(int(0.05*min(edges.shape)), 1)
                cv2.line(edges,(x1,y1),(x2,y2),(0,0,0),line_thickness)
                cv2.line(debug_img,(x1,y1),(x2,y2),(0,255,0),2)
        except TypeError:
            break
    if debug:
        cv2.imshow("remaining edges", edges)
        cv2.waitKey(0)
        pass

    if (len(lines) < 2):
        return None, None
    elif (len(lines) < 4):
        #if there arent 4 lines we can assume the lane marker is straight i.e. we only consider two lines
        if len(lines) == 3:
            #remove line that is most different from the other two
            l0l1_angle = angleBetweenLines(lines[0], lines[1])
            l0l2_angle = angleBetweenLines(lines[0], lines[2])
            l1l2_angle = angleBetweenLines(lines[1], lines[2])
            if min(l0l1_angle, l0l2_angle, l1l2_angle) == l1l2_angle:
                lines.pop(0)
            elif min(l0l1_angle, l0l2_angle, l1l2_angle) == l0l2_angle:
                lines.pop(1)
            else:
                lines.pop(2)
        #get slope
        s1 = lines[0]
        s2 = lines[1]
        #average the lines to get better approximation of actual line
        s1_angle = ((180*math.atan(s1[0])/math.pi) % 180)
        s2_angle = ((180*math.atan(s2[0])/math.pi) % 180)
        #ensure that angles are within 90 degrees of each other so we always average the acute angle between lines
        if abs(s1_angle-s2_angle) > 90:
            s1_angle = s1_angle-180
        #negate since y starts at top of image
        avg_angle = -(s1_angle+s2_angle)/2
        avg_slope = math.tan(math.pi*avg_angle/180)
        
        angle1 = avg_angle
        if angle1 > 0: angle2 = angle1-180
        else: angle2 = angle1+180

        finalLines = [angle1,angle2]

        M = cv2.moments(255-thresh_img)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centerPoint = (cX, cY)

        return finalLines, centerPoint
    else:
        #combine the 4 lines into 2 most different lines
        finalLines = []
        #array to hold the 4 lines, organized by heading (2 lines per heading, upper and lower)
        linesToFindCenter = []
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
            linesToFindCenter.append((upper_line,lower_line))
        
        #get the center point of the lane marker using the rectangle defined by the 4 lines on the lane markers edges
        centerPoint = getCenterPoint(linesToFindCenter)
        if centerPoint == None: return None, None
        avgs = []
        for slope in finalLines:
            dilation_kernel = np.ones((5,5), np.uint8)
            inverted_thresh_img = cv2.bitwise_not(thresh_img)
            while (not inverted_thresh_img[int(centerPoint[1])][int(centerPoint[0])] == 255):
                inverted_thresh_img = cv2.dilate(inverted_thresh_img, dilation_kernel, iterations=1)
            thresh_img = cv2.bitwise_not(inverted_thresh_img)
            negStepsWithLM = getStepsWithLM(thresh_img, slope, -1, centerPoint)
            posStepsWithLM = getStepsWithLM(thresh_img, slope, 1, centerPoint)
            avgs.append([slope, posStepsWithLM, 1])
            avgs.append([slope, negStepsWithLM, -1])
        s1 = max(avgs, key=lambda x : x[1])
        avgs.remove(s1)
        s2 = max(avgs, key=lambda x : x[1])
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

def getStepsWithLM(img, slope, direction, center_point, lm_color=0):
    x,y = center_point
    num_steps = 50
    if abs(slope) > 1: #slopes that change faster in y than x 
        step_y = img.shape[0]/(2*num_steps)
        if slope < 0: step_y *= -1.0 #step_y must always have same sign as the slope
        step_x = abs(step_y/slope) # step_x must always be positive
    else:
        step_x = img.shape[1]/(2*num_steps)
        step_y = slope*step_x
        
    stepsWithLM = 0
    while (x < img.shape[1] and y < img.shape[0] and x>0 and y>0):
        if img[int(y)][int(x)] == lm_color: stepsWithLM += 1
        x += step_x*direction
        y += step_y*direction
    return stepsWithLM

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
        cv2.line(img, center_point, line_end, (255, 0, 0), line_thickness)
        #add text with measured angle of line at the end of the line
        cv2.putText(
            img,
            text=str(angle) + " deg.",
            org=line_end,
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.4, 
            color= (255, 0, 0), 
            lineType=cv2.LINE_AA,
        )
    cv2.circle(img, center_point, radius=5, color=(255, 0, 0), thickness=-1)
    return img

pwd = os.path.realpath(os.path.dirname(__file__))
if __name__ == '__main__':
    #run this script to see the heading detection step by step
    for img_i in [1,2,3,4,5,6,7]:
        test_image_filename = pwd + "/images/lm (" + str(img_i) + ") straight.png"
        img = cv2.imread(test_image_filename)
        visualizeLaneMarker(img, debug=False)
        cv2.imshow("visualization " + str(img_i), img)
        cv2.waitKey(0)
        test_image_filename = pwd + "/images/lm (" + str(img_i) + ").png"
        img = cv2.imread(test_image_filename)
        visualizeLaneMarker(img, debug=False)
        cv2.imshow("visualization " + str(img_i), img)
        cv2.waitKey(0)
