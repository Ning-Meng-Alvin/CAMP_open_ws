#!/usr/bin/env python
'''
Author: Dianye Huang (dianye.huang@tum.de)
Date: 2022-05-05 16:07:48
LastEditors: Dianye Huang
LastEditTime: 2023-05-20 15:13:41
Description: 
    This script streams the ultrasound(US) image from the US machine via a framegrabber. 
    One can configure the capturing setting through the rqt_reconfigure plugin. 
    The reconfigurable parameters include: video index, cropping area, horizontal flip 
    the cropped image, color or grey image. The streamed image is then pulished as a 
    rostopic with the topic_name and frame_id of the message being set in "cfg/config.yaml"
    
    - This script uses cv2 module to capture the vedio channels 
    - The image is published by rostopic named 'frame_grabber/us_img' by default
    - To use the auto_crop function robustly, make sure to increase the gain to the highest value 

Notes:
    - prior settings of Siemens Healthineers (init_cbox)
        Sie_x0 = 500
        Sie_y0 = 180
        Sie_x1 = 1420
        Sie_y1 = 850
    - cbox = [x0, x1, y0. y1]
'''

import os
import cv2
import yaml
import numpy as np

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from dynparam_utils import FGDynParamRecfg

class FrameGrabber:
    def __init__(self, node_name, yaml_filepath):
        
        # Dynamic reconfigure server
        rospy.init_node(node_name, anonymous=False)
        self.fgdyn_cfg = FGDynParamRecfg(yaml_filepath=yaml_filepath + '.yaml')
        
        # Load settiings
        self.yaml_filepath = yaml_filepath
        self.capture_config = self.load_yaml(yaml_filepath + '.yaml')
        self.pub_topic = self.capture_config['pub_topic']
        self.frame_id = self.capture_config['frame_id']
        
        # Denoting a possible region for cropping
        self.init_cbox = (
                        self.capture_config['init_cbox']['x0'],
                        self.capture_config['init_cbox']['x1'],
                        self.capture_config['init_cbox']['y0'],
                        self.capture_config['init_cbox']['y1']
        ) 
        self.final_cbox = (
                        self.capture_config['final_cbox']['x0'],
                        self.capture_config['final_cbox']['x1'],
                        self.capture_config['final_cbox']['y0'],
                        self.capture_config['final_cbox']['y1']
        ) 
        self.video_index = self.capture_config['video_index']
        
        # choose video stream index
        self.cap = None
        self.video_index_seq = 0
        ret, self.video_index_list = self.get_video_idx(max_num=10)
        if not ret:
            print('no video stream available!')
        else:
            self.num_video_stram_index = len(self.video_index_list)
            print('available index: ', self.video_index_list)
            
            self.cap = self.init_video_cap(self.video_index)
            if not self.cap.isOpened():
                self.video_index = self.video_index_list[self.video_index_seq]
                self.cap = self.init_video_cap(self.video_index)
        
        # publish image
        self.bridgeC = CvBridge()
        self.pub_img = rospy.Publisher(self.pub_topic, Image, queue_size=1)
        
    def run(self):
        img_seq = 0
        flag_show = False
        flag_save_img = False
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
            # dynamic reconfigure the cropping area
            if self.fgdyn_cfg.cfg_flag:
                self.fgdyn_cfg.cfg_flag = False
                if self.fgdyn_cfg.trigger_update_list:
                    self.fgdyn_cfg.trigger_update_list = False
                    ret, self.video_index_list = self.get_video_idx(max_num=10)
                    if not ret:
                        print('no video stream available!')
                    else:
                        self.video_index_list.append(self.video_index)
                        self.num_video_stram_index = len(self.video_index_list)
                        print('available index: ', self.video_index_list)
                    
                if self.fgdyn_cfg.trigger_prev_idx:
                    self.fgdyn_cfg.trigger_prev_idx = False
                    if self.video_index_list is not None:
                        self.video_index_seq += 1
                        self.video_index_seq %= self.num_video_stram_index
                        self.cap.release()
                        self.video_index_list.append(self.video_index)
                        self.video_index = self.video_index_list[self.video_index_seq]
                        self.cap = self.init_video_cap(self.video_index)
                
                if self.fgdyn_cfg.trigger_next_idx:
                    self.fgdyn_cfg.trigger_next_idx = False
                    if self.video_index_list is not None:
                        self.video_index_seq -= 1
                        self.video_index_seq %= self.num_video_stram_index
                        self.cap.release()
                        self.video_index_list.append(self.video_index)
                        self.video_index = self.video_index_list[self.video_index_seq]
                        self.cap = self.init_video_cap(self.video_index)
                
                if self.fgdyn_cfg.trigger_save_settings:
                    self.fgdyn_cfg.trigger_save_settings = False
                    self.capture_config['hflip'] = self.fgdyn_cfg.hflip_image
                    self.capture_config['color'] = self.fgdyn_cfg.color_image
                    self.capture_config['time_offset'] = self.fgdyn_cfg.time_offset
                    self.capture_config['final_cbox']['x0'] = self.cbox[0]
                    self.capture_config['final_cbox']['x1'] = self.cbox[1]
                    self.capture_config['final_cbox']['y0'] = self.cbox[2]
                    self.capture_config['final_cbox']['y1'] = self.cbox[3]
                    self.capture_config['video_index'] = self.video_index
                    self.save_yaml(self.yaml_filepath + '.yaml', self.capture_config)
                    flag_save_img = True
                    
                if self.fgdyn_cfg.auto_cropping:
                    self.fgdyn_cfg.cfg_flag = True
                    grey_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
                    ret, cbox = self.get_cropping_param(grey_img)
                    if ret is True:
                        self.cbox = cbox
                        self.fgdyn_cfg.set_cbox(self.cbox)
                        # show results
                        frame = cv2.rectangle(frame, (cbox[0], cbox[2]), 
                                        (cbox[1], cbox[3]), (0, 0, 255))
                        frame = cv2.resize(frame, None, fx=0.8, fy=0.8, 
                                        interpolation=cv2.INTER_AREA)
                        cv2.imshow('frame', frame)
                        cv2.waitKey(2)
                        flag_show = True
                    else:
                        print('Failed to automatically crop the screen shot!')
            else:
                if flag_show:
                    flag_show = False
                    cv2.destroyAllWindows()
            
            # publish images
            if not self.cap.isOpened(): 
                continue
            ret, frame = self.cap.read()
            if ret and not self.fgdyn_cfg.auto_cropping:
                # save preprocessing result
                if flag_save_img:
                    flag_save_img = False
                    frame_vis = cv2.rectangle(frame, (self.cbox[0], self.cbox[2]), 
                                        (self.cbox[1], self.cbox[3]), (0, 0, 255))
                    frame_vis = cv2.resize(frame_vis, None, fx=0.8, fy=0.8, 
                                    interpolation=cv2.INTER_AREA)
                    cv2.imwrite(self.yaml_filepath + '.png', frame_vis)
                
                # preprocess image
                self.cbox = self.fgdyn_cfg.get_cbox()
                height, width = frame.shape[:2]
                self.cbox[1] = min(self.cbox[1], width)
                self.cbox[3] = min(self.cbox[3], height)
                
                frame = frame[self.cbox[2]:self.cbox[3], 
                                self.cbox[0]:self.cbox[1]]
                
                if not self.fgdyn_cfg.color_image:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                if self.fgdyn_cfg.hflip_image:
                    frame = np.flip(frame, axis=1)
                
                # publish image
                msg_img = self.bridgeC.cv2_to_imgmsg(frame)
                msg_img.header.frame_id = self.frame_id
                msg_img.header.seq = img_seq
                img_seq += 1

                # Apply time offset compensation for system latency
                # Use dynamic reconfigure parameter for time offset
                time_offset = rospy.Duration(self.fgdyn_cfg.time_offset)
                msg_img.header.stamp = rospy.get_rostime() - time_offset
                self.pub_img.publish(msg_img)
            
        if self.cap is not None:
            self.cap.release()

                
    # def tmp(self):
        # # Choose a video index
        # self.vid_index = None
        # ret, self.vid_idxs = self.get_video_idx()
        # if ret is False:
        #     print('No available streams!')
        #     return
        # for idx in self.vid_idxs:
        #     if idx == tmp_vid_index:
        #         self.vid_index = idx
        #         break
        # if self.vid_index is None:
        #     self.vid_index = self.vid_idxs[-1]
        
        # # Automatically crop the US image
        # self.cap = self.init_video_cap(self.vid_index)
        # while True:
        #     ret, frame = self.cap.read()
        #     if ret is True:
        #         grey_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # color or grey
        #         ret, cbox = self.get_cropping_param(grey_img)
        #         if ret is True:
        #             # save result
        #             self.final_cbox = cbox
        #             self.capture_config['final_cbox']['x0'] = cbox[0]
        #             self.capture_config['final_cbox']['x1'] = cbox[1]
        #             self.capture_config['final_cbox']['y0'] = cbox[2]
        #             self.capture_config['final_cbox']['y1'] = cbox[3]
                    
        #             # show results
        #             frame = cv2.rectangle(frame, (cbox[0], cbox[2]), 
        #                             (cbox[1], cbox[3]), (0, 0, 255))
        #             frame = cv2.resize(frame, None, fx=0.8, fy=0.8, 
        #                             interpolation=cv2.INTER_AREA)
        #             cv2.imshow('frame', frame)
        #         else:
        #             print('Failed to automatically crop the screen shot!')
                    
        #     cv2.waitKey(2)
            
        # self.cap.release()
        
        # Publish cropped image
        # - color or grey
        # - horizontally flip or not
        

# cropped images
# cropped_frame = np.array(frame[cbox[2]:cbox[3], cbox[0]:cbox[1]])
# cv2.imshow('cropped_frame', cropped_frame)

    def load_yaml(self, filepath):
        with open(filepath, 'r') as f:
            capture_config = yaml.load(f.read(), Loader=yaml.FullLoader)
        return capture_config
    
    def save_yaml(self, filepath, yaml_dict):
        with open(filepath, 'w') as f:
            yaml.dump(yaml_dict, f)
    
    def get_cropping_param(self, grey_img):
        # Automatic cropping, the input is a grey image
        # 1. Remember to set the gain of the US dev to be 90 dB
        # 2. The pixel coordinate is defined as:
        #       (x0, y0) -> upper left corner
        #       (x1, y1) -> lower right corner
        init_crop = np.array(grey_img[self.init_cbox[2]:self.init_cbox[3], 
                                self.init_cbox[0]:self.init_cbox[1]]) 
        mask = init_crop.copy() * 0 + 10
        mask[np.where(init_crop==0)] = 0
        
        # check the histogram to determine the cropped area
        lr_hist = np.sum(mask, axis=0)   # decide x0 and x1
        ud_hist = np.sum(mask, axis=1)   # decide y0 and y1
        
        # for the left and right lines
        x0_fs = x1_fs = y1_fs = 0
        for idx, val in enumerate(lr_hist):
            if val > 500:
                x0_fs = idx
                break
        for idx, val in enumerate(np.flip(lr_hist)):
            if val > 500:
                x1_fs = idx+1
                break
        # for the bottom line
        for idx, val in enumerate(np.flip(ud_hist)):
            if val > 500:
                y1_fs = idx+1
                break
        if y1_fs == 0:
            return False, None
        
        x0, x1 = self.init_cbox[0] + x0_fs, self.init_cbox[1] - x1_fs
        y0, y1 = self.init_cbox[2], self.init_cbox[3] - y1_fs
        
        return True, [x0, x1, y0, y1]
    
    def init_video_cap(self, video_index):
        # Use V4L2 backend to avoid GStreamer issues
        video_cap = cv2.VideoCapture(video_index, cv2.CAP_V4L2)
        if not video_cap.isOpened():
            # If V4L2 fails, try default backend
            video_cap = cv2.VideoCapture(video_index)
        
        if video_cap.isOpened():
            # Set resolution
            video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
            # Set buffer size
            video_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            print(f"Video capture initialized successfully for device {video_index}")
        else:
            print(f"Failed to initialize video capture for device {video_index}")
        
        return video_cap
    
    def get_video_idx(self, max_num=10):
        # find the index of available video devices
        video_stream_index = []
        for i in range(max_num): # TODO 10 is supposed to be configurable
            tmp = cv2.VideoCapture(i)
            if tmp.isOpened():
                video_stream_index.append(i)
                tmp.release()
        num_video_stram_index = len(video_stream_index)
        if num_video_stram_index == 0:
            print("no video data is available!")
            return False, None
        return True, video_stream_index
    


if __name__ == '__main__':
    fg = FrameGrabber(
            node_name='dyncfg_framegrabber',
            yaml_filepath=os.path.dirname(os.path.dirname(__file__))+'/cfg/config',
        )
    fg.run()
    print('hello world!')

