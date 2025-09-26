#!/usr/bin/env python
'''
Author: Dianye Huang (dianye.huang@tum.de)
Date: 2023-05-18 10:40:13
LastEditors: Dianye Huang
LastEditTime: 2023-05-20 10:28:32
Description: 
    Server for dynamic reconfigure
    relating files: 
        - "cfg/config.yaml"
        - "cfg/FGDynParam.cfg"
Notes:
    1. To change the parameters, rosrun the following command
        $ rosrun rqt_gui rqt_gui -s reconfigure
            or 
        $ rosrun rqt_reconfigure rqt_reconfigure
    2. Parameter description (cfg/FGDynParam.cfg):
        prompt            : prompt of GUI event 
        update_index_list : trigger signal, Update the list of available video stream
        prev_video_index  : trigger signal, Previous video index
        next_video_index  : trigger signal, Next video index
        auto_cropping     : switch signal, automatically detect the US image within the
                            init box indicated in the (cfg/config.yaml) file
        hflip_image       : switch signal, horizontally switch the streamed US image
        color_image       : switch signal, color or grey image
        save_settings     : trigger signal, save configuration to the (cfg/config.yaml) file
''' 
import rospy
import yaml

from dynamic_reconfigure.server import Server
from usdev_framegrab.cfg import FGDynParamConfig

class FGDynParamRecfg:
    '''
    Should be called after init a ros node
    '''
    def __init__(self, yaml_filepath):
        self.dynrcfg_prompt = None
        self.auto_cropping  = False
        self.hflip_image = False
        self.color_image = False
        self.time_offset = 0.0  # time offset in seconds
        self.cbox = None # cropped box (x0, x1, y0, y1)
        self.cfg_flag = False
        self.trigger_update_list = False
        self.trigger_prev_idx = False
        self.trigger_next_idx = False
        self.trigger_save_settings = False
        self.yaml_filepath = yaml_filepath
        Server(FGDynParamConfig, self.server_callback)
    
    def set_cbox(self, box):
        self.cbox = box
    
    def get_cbox(self):
        return self.cbox
    
    def server_callback(self, config, level):
        # prompt
        if self.dynrcfg_prompt is None:
            self.dynrcfg_prompt = config.prompt
            # init config according to the yaml file
            with open(self.yaml_filepath, 'r') as f:
                capture_config = yaml.load(f.read(), 
                                    Loader=yaml.FullLoader)
            config.hflip_image = capture_config['hflip']
            config.color_image = capture_config['color']
            config.time_offset = capture_config.get('time_offset', 0.0)
            config.topleft_x0 = capture_config['final_cbox']['x0']
            config.topleft_y0 = capture_config['final_cbox']['y0']
            config.bottomright_x1 = capture_config['final_cbox']['x1']
            config.bottomright_y1 = capture_config['final_cbox']['y1']
            self.hflip_image = config.hflip_image
            self.color_image = config.color_image
            self.time_offset = config.time_offset
            self.cbox =[config.topleft_x0, config.bottomright_x1,
                        config.topleft_y0, config.bottomright_y1]
            
        if level != 0:
            self.cfg_flag = True
        
        # trigger signals
        if level == 1:
            if config.update_index_list is True:
                config.update_index_list = False
                self.dynrcfg_prompt = 'trigger update index list'
                self.trigger_update_list = True
                
            if config.prev_video_index is True:
                config.prev_video_index = False
                self.dynrcfg_prompt = 'trigger prev video index'
                self.trigger_prev_idx = True
                
            if config.next_video_index is True:
                config.next_video_index = False
                self.dynrcfg_prompt = 'trigger next video index'
                self.trigger_next_idx = True
        
        if level == 2: # x0
            if config.topleft_x0 > config.bottomright_x1:
                config.topleft_x0 = config.bottomright_x1 - 2
            self.cbox[0] = config.topleft_x0
            self.dynrcfg_prompt = 'manually update x0'
        
        if level == 4: # x1
            if config.bottomright_x1 < config.topleft_x0:
                config.bottomright_x1 = config.topleft_x0 + 2
            self.cbox[1] = config.bottomright_x1
            self.dynrcfg_prompt = 'manually update x1'
            
        if level == 3: # y0
            if config.topleft_y0 > config.bottomright_y1:
                config.topleft_y0 = config.bottomright_y1 - 2
            self.cbox[2] = config.topleft_y0
            self.dynrcfg_prompt = 'manually update y0'
            
        if level == 5: # y1
            if config.bottomright_y1 < config.topleft_y0:
                config.bottomright_y1 = config.topleft_y0 + 2
            self.cbox[3] = config.bottomright_y1
            self.dynrcfg_prompt = 'manually update y1'
        
        if level == 6: 
            self.auto_cropping = config.auto_cropping
            if self.auto_cropping is True:
                self.dynrcfg_prompt = 'auto update cropped area'
            else:
                self.dynrcfg_prompt = 'stop update cropped area'
                # change the cropping settings
                config.topleft_x0 = self.cbox[0]
                config.topleft_y0 = self.cbox[2]
                config.bottomright_x1 = self.cbox[1]
                config.bottomright_y1 = self.cbox[3]
        
        if level == 7:
            self.hflip_image = config.hflip_image
        
        if level == 8:
            self.color_image = config.color_image
        
        if level == 10:
            self.time_offset = config.time_offset
            self.dynrcfg_prompt = f'time offset set to {self.time_offset:.3f}s'
        
        if level == 9:
            if config.save_settings is True:
                config.save_settings = False
                self.dynrcfg_prompt = 'trigger save settings'
                self.trigger_save_settings = True
        
        config.prompt = self.dynrcfg_prompt
        return config


if __name__ == "__main__":
    rospy.init_node("framegrabber_params", anonymous = False) 
    fgdyn_server = FGDynParamRecfg()
    rospy.spin()