#!/usr/bin/env python
import sys
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QGraphicsPixmapItem, QMainWindow, QApplication, QWidget
from PyQt5.QtWidgets import QPushButton, QLabel, QGraphicsView, QGraphicsScene
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QGridLayout

# ros
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
# opencv2
import cv2
from cv_bridge import CvBridge

import numpy as np

class calib_gui(QMainWindow):
    def __init__(self, wide, high, file_path):
        super().__init__()
        
        # ros init
        # images
        rospy.init_node('handeye_calibration', anonymous=True)
        
        # self.cimg_sub = rospy.Subscriber("/camera/color/image_raw", Image, 
        #                                 self.sub_img_cb)
        # self.bridgeC = CvBridge()
        # self.img = None
        # rospy.wait_for_message("/camera/color/image_raw", Image, timeout=3)

        # joint states
        self.cimg_sub = rospy.Subscriber("/franka_state_controller/joint_states", 
                                        JointState, self.sub_jpos_cb)
        self.jpos = None
        rospy.wait_for_message("/franka_state_controller/joint_states", JointState, timeout=3)
        self.log_cnt = 0

        self.prompt_str = "Do you want to record the current joint states?\n" + \
                        "- Press Next to confirm recording joint states\n" + \
                        "- Press Quit to exit the program"
        
        # gui init
        self.wide = wide
        self.high = high
        self.initUI()
        # self.img_timer = QTimer()
        # self.img_timer.timeout.connect(self.show_imgs)  # connect signals and slot
        # self.img_timer.start(30) # 30 Hz

        self.jpos_timer = QTimer()
        self.jpos_timer.timeout.connect(self.show_jpos)
        self.jpos_timer.start(30) 

        # log files
        # path_prefix = "/home/hdy/franka_oa/franka_oa_ws/src/franka_handeye_cali/config/"
        # log_joint_pos_name = "joint_cmds.txt"
        self.log_joints_file = file_path
        with open(self.log_joints_file, 'w') as f:
            f.write('q1'+'\t' + 'q2'+'\t' + 'q3'+'\t' + \
                    'q4'+'\t' + 'q5'+'\t' +'q6'+'\t' +'q7'+'\n')

    def show_imgs(self):
        y, x = self.img.shape[:2]
        frame = QImage(self.img, x, y, QImage.Format.Format_RGB888)
        self.pix = QPixmap.fromImage(frame)
        self.item=QGraphicsPixmapItem(self.pix)
        self.scene = QGraphicsScene()
        self.scene.addItem(self.item)
        self.imgshow.setScene(self.scene)
        
    def show_jpos(self):
        jstr = '<b>Current joint position (rad):</b>'
        for j in self.jpos:
            j = round(j, 3)
            jstr += '\t' + str(j)
        self.jinfo.setText(jstr)

    def sub_img_cb(self, ros_img):
        img = self.bridgeC.imgmsg_to_cv2(ros_img, "bgr8")
        self.img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    def sub_jpos_cb(self, ros_msg):
        self.jpos = ros_msg.position


    def on_click_next(self):
        self.log_cnt += 1
        self.prompt.setText('Have recorded '+ str(self.log_cnt) + 
                            ' joint command\n' + self.prompt_str)
        with open(self.log_joints_file, 'a') as f:
            for idx, j in enumerate(self.jpos):
                j = round(j, 4)
                if idx == 6:
                    f.write(str(j))
                else:
                    f.write(str(j)+'\t')
            f.write('\n')
    
    def initUI(self):
        # setting init geometry of GUI
        # self.set_geometry(self.wide, self.high)

        # init widgets
        # 1. menu
        # pass

        # 2. buttons
        # -1 next button
        self.btn_next = QPushButton('next', self)
        self.btn_next.clicked.connect(self.on_click_next)
        # -2 quit button
        self.btn_quit = QPushButton('quit', self)
        self.btn_quit.clicked.connect(QApplication.instance().quit)

        # 3. labels
        self.prompt = QLabel(self.prompt_str)
        self.jinfo = QLabel('<b>Current joint position (rad):</b>')

        # 4. graphics
        self.imgshow = QGraphicsView()
        self.imgshow.setObjectName('realsense_raw_color_imgs')

        # add status bar
        self.statusBar().showMessage('Ready')

        # 3. setup layout
        self.hl_button = QHBoxLayout()
        self.hl_button.addSpacing(10)
        self.hl_button.addStretch(1)
        self.hl_button.addWidget(self.btn_next)
        self.hl_button.addWidget(self.btn_quit)

        self.layout = QGridLayout()
        self.layout.setSpacing(5)
        # self.layout.addWidget(self.imgshow, 1, 0)
        self.layout.addWidget(self.jinfo,  2, 0)
        self.layout.addWidget(self.prompt, 3, 0)
        self.layout.addLayout(self.hl_button, 4, 0)

        # setup Qwidget in mainWindow
        widget = QWidget()
        widget.setLayout(self.layout)
        self.setCentralWidget(widget)

        self.setWindowTitle('Franka Handeye Calibration Log cmd GUI')
        self.show()
    
    def set_geometry(self, wide, high):
        self.resize(wide, high)
        qr = self.frameGeometry()
        cp = self.screen().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        

if  __name__ == '__main__':
    cmd_log_path = sys.argv[1]
    app = QApplication([])
    gui = calib_gui(wide=800, high=700, file_path=cmd_log_path)
    sys.exit(app.exec())
    
    
