# extract images from rosbag

import os
import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm

class BagToImage(object):

    def __init__(self, bagfile_path, camera_topic, folder):
        self.bagfile_path = bagfile_path  # bag path
        self.camera_topic = camera_topic  # topics to be recorded
        self.image_dir = folder # root path to save the images
        
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)

    def extractFromCamera(self):
        bag = rosbag.Bag(self.bagfile_path, "r")  # read rosbag
        bridge = CvBridge()  # 用于将图像消息转为图片
        bag_data_imgs = bag.read_messages(self.camera_topic)  # 读取图像消息

        index = 0

        pbar = tqdm(bag_data_imgs)
        for topic, msg, t in pbar:
            pbar.set_description("Processing extract image id: %s" % (index + 1))
            # 消息转为图片
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  
            # 存储图片
            cv2.imwrite(os.path.join(self.image_dir, str(index) + ".bmp"), cv_image)
            index += 1


if __name__ == '__main__':
    bagfile_path = "/home/xxx/Code/data/xxx.bag"  # 此处为绝对路径

    camera_topic = "/camera_compress"  # 准备工作中得到的topic

    extract_image = BagToImage(bagfile_path, camera_topic, "xxx")  # xxx处路径就是存放照片的路径

    extract_image.extractFromCamera()
    
    
    
    