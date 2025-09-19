'''
Author: Dianye Huang
Date: 2024-02-22 13:49:00
LastEditors: Dianye Huang
LastEditTime: 2024-02-22 14:07:12
Description: 
'''

import os
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class GrabImgFromTopic:
    def __init__(self, topic_name):
        self.cvbridge = CvBridge()
        self.flag_img_update = False
        rospy.Subscriber(topic_name, Image, self.usimg_cb, queue_size=1)

        
    def get_img(self):
        while not self.flag_img_update: pass
        self.flag_img_update = False
        return self.usimg
            
    
    def usimg_cb(self, msg:Image):
        self.usimg = self.cvbridge.imgmsg_to_cv2(msg)
        self.flag_img_update = True


if __name__ == '__main__':
    gift = GrabImgFromTopic(
        topic_name = '/frame_grabber/us_img'
        )
    
    rospy.init_node('save_img', anonymous=False)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
        img = gift.get_img()
        
        # save_img
        savepath = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'scripts/depth_11cm.jpg'
        )
        cv2.imwrite(savepath, img.astype(np.uint8))
        break
        
        # # display img
        # cv2.imshow('hello', img.astype(np.uint8))
        # cv2.waitKey(30)
    
    


