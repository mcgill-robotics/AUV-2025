#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Image
#from std_msgs.msg import Boolean



def image_cb(image_msg):

  width = image_msg.width
  height = image_msg.height

  img_data = list(image_msg.data) 


  img_matrix = np.array(img_data)
  img_matrix = np.reshape(img_matrix, (width, height, -1))

  
  pixel_blue = img_matrix[:,:,0]

          
  print(np.all(pixel_blue[pixel_blue >  200]))



if __name__ == '__main__':
    print("detector")
    rospy.init_node('detector')
    image_sub = rospy.Subscriber('/image_publisher_1651798201951158778/image_raw', Image, image_cb, queue_size=50)
    #pub = rospy.Publisher('object_detected', Boolean, queue_size=50)
    rospy.spin()
