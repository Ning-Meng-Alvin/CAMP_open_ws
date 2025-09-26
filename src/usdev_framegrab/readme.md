<!--
 * @Author: Dianye Huang
 * @Date: 2023-05-18 11:23:42
 * @LastEditors: Dianye Huang
 * @LastEditTime: 2023-05-20 10:48:50
 * @Description: 
   * readme.md
-->

# 0. introduction
This script streams the ultrasound(US) image from the US machine via a framegrabber. One can configure the capturing setting through the rqt_reconfigure plugin. The reconfigurable parameters include: video index, cropping area, horizontal flip the cropped image, color or grey image. The crpped image is wrapped as a ros message and published to the ros master.

# 1. ros package preparation
- **Step 1**: if you are using a Maxwell framegrabber, download the configuration tools / driver from
https://www.magewell.com/downloads/usb-capture-plus#/tools/linux-ubuntu and type the following command:
``` bash
sudo usbcaptureutility
```
- **Step 2**: copy and paste this package to your ros workspace and compile the workspace

# 2. run the package
run the launch file to make some configurations
``` bash
roslaunch usdev_framegrab stream_usimg.launch  

or

roslaunch usdev_framegrab stream_usimg_dyncfg.launch
```

# 3. adjust the streaming settings
type the following command into the terminal
``` bash
rosrun rqt_reconfigure rqt_reconfigure
```
The cropping, flipping, greying of the streamed image can be configured in rqt_reconfigure pulgin ($ rosrun rqt_reconfigure rqt_reconfigure).
The topic_name and the frame_id of the ros message can be modified in "cfg/config.yaml"

# Notes:
## Define a param in *.cfg for dynamic reconfigure
- (name, paramtype, level, description, default, min, max)
## Auto Detecting the region of ultrasound image
- To use the auto_crop function robustly, make sure to increase the gain in the US machine to the highest value (e.g. 90 dB for SIEMENS Healthineers)
