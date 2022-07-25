#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from lane_marker.srv import LanesInfo, LanesInfoRequest
import rospkg
import cv2

path = rospkg.RosPack().get_path('lane_marker')
image = cv2.imread(path + '/images/source.png')
rospy.init_node('image_service_client',anonymous=True)
bridge = CvBridge()
ros_img = bridge.cv2_to_imgmsg(image,'bgr8')
rospy.wait_for_service('lanes_info')

try:
    proxy = rospy.ServiceProxy('lanes_info',LanesInfo)
    resp = proxy(ros_img)
    print('Center at:\n',resp.path_marker_center)
    print('Heading from last task:\n',resp.path_marker_heading_from)
    print('Heading to next task:\n',resp.path_marker_heading_to)
    imgResult = bridge.imgmsg_to_cv2(resp.path_marker_result_img)
    cv2.imshow('Lanes',image)
    cv2.imshow('Result',imgResult)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()
except rospy.ServiceException as e:
    print(str(e))
