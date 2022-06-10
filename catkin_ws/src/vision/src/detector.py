#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool



def image_cb(image_msg):

    width = image_msg.width
    height = image_msg.height
    
    print(image_msg.encoding)

    print("step value: ",image_msg.step,", width val:",image_msg.width,", height val:",image_msg.height,", data points:",len(image_msg.data))

    img_data = list(image_msg.data)
        
    #encoding = bgr, 0=b 1=g 2=r value in range below   
    for i in range(2,len(image_msg.data),3):
        if img_data[i] > 150:
            print("red")


if __name__ == '__main__':
    print("detector")
    
    rospy.init_node('detector')
    
    image_sub = rospy.Subscriber('/image_publisher/image_raw', Image, image_cb, queue_size=50) 
    
    pub = rospy.Publisher('object_detected', Bool, queue_size=50)
    
    rospy.spin()
