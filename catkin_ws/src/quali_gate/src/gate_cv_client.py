#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from quali_gate.srv import GateInfo, GateInfoRequest
import rospkg

path = rospkg.RosPack().get_path('quali_gate')
original = cv2.imread(path + '/images/gate.png')
bridge = CvBridge()
rosImg = bridge.cv2_to_imgmsg(original)
rospy.init_node('gate_client', anonymous=True)
rospy.wait_for_service('gate_info')

try:
    proxy = rospy.ServiceProxy('gate_info', GateInfo)
    resp = proxy(rosImg)
    print('Gate center at: ',resp.gate_center.x, ',', resp.gate_center.y)
    print('Frame center at: ',resp.frame_center.x, ',', resp.frame_center.y)
    imgResult = bridge.imgmsg_to_cv2(resp.result_image)
    cv2.imshow('Lanes', original)
    cv2.imshow('Result', imgResult)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()
except rospy.ServiceException as e:
    print(str(e))