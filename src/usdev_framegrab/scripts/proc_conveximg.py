'''
Author: Dianye Huang
Date: 2024-02-22 15:04:20
LastEditors: Dianye Huang
LastEditTime: 2024-02-22 18:05:17
Description:
'''


import cv2
import rospy
import numpy as np
import matplotlib.pyplot as plt

"""
Params for C51

The cropping params are:
final_cbox:
    x0: 521, x1: 1399
    y0: 180, y1: 837
frame_id: us_prob
hflip: false
init_cbox:
    x0: 500, x1: 1420
    y0: 180, y1: 850

Resolution: 657 x 878 (height x width), 
probe tip in depth: 07.3 mm
probe tip in width: 46.4 mm
7  cm: headline edge gap 188 pixels, top gap 62 pixels   689
8  cm: headline edge gap 216 pixels, top gap 55 pixels   661
9  cm: headline edge gap 239 pixels, top gap 49 pixels   638
10 cm: headline edge gap 258 pixels, top gap 45 pixels   619
11 cm: headline edge gap 273 pixels, top gap 41 pixels   604
"""

if __name__ == '__main__':
    image_depth = 11 # unit: cm
    
    emit_angle  = 70 # unit: deg
    img_gaps = {'7': [62, 188], '8': [55, 216], '9': [49, 239], '10': [45, 258], '11': [41, 273]}
    
    img_hgap, img_wgap = img_gaps[str(int(image_depth))]
    imgpath = '/home/hdy/Projects_ws/robot_learning/src/record_dataset/usdev_framegrab/scripts/depth_' + \
                str(int(image_depth)) + 'cm.jpg'
    img = cv2.imread(imgpath)[:,:,0]
    imgshape = img.shape
    
    # headline = img[0, :]
    # midline = img[:, imgshape[1]//2]
    # print('imgshape: ', imgshape)
    # print('headline shape: ', headline.shape)
    # print('midline shape: ', midline.shape)
    # plt.figure()
    # plt.plot(headline) # midline
    # plt.grid(True)
    # plt.show()
    
    tmp_rad = emit_angle/2/180.0*np.pi
    hres = image_depth*10 / (float(imgshape[0])-img_hgap) # unit: mm/px
    wres = img_hgap*hres/(1-np.cos(tmp_rad))*np.sin(tmp_rad)/ \
                        (float(imgshape[1])/2.0-img_wgap) # uint: mm/px
    hphy = hres*imgshape[0]
    wphy = wres*imgshape[1]
    print('------------------------------------')
    print('imaging depth: ', image_depth , 'cm')
    print('hres (mm/px): ', hres, ', wres(mm/px): ', wres)
    print('fixed hgap (mm): ', hres*img_hgap)
    print('fixed wgap (mm): ', wres*(float(imgshape[1])-2*img_wgap))
    print('physical size (height x width, mm): ', hphy, 'x',wphy)
    print('------------------------------------')
    

    # subimg = img[:100, :]
    cv2.imshow('hellow world', img)
    cv2.waitKey(3000)
    cv2.destroyAllWindows()



