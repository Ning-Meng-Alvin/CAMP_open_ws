#!/usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray

import sys
import time
from geometry_msgs.msg import WrenchStamped

class ForceCaliDataAcq(object):
  def __init__(self, path_prefix, end_effector_link):
    rospy.loginfo("Data acquisition for force calibration")
    # global variables
    self.Fext = []
    self.rate  = rospy.Rate(1)  # Hz
    self.rate2 = rospy.Rate(50) 
    self.ee_link = end_effector_link
    self.commander = MoveGroupCommander('panda_arm')
    _sub_fext = rospy.Subscriber("/netft_data", 
                                  WrenchStamped, self.sub_F_ext_cb,
                                  queue_size=1, tcp_nodelay=True)
    self.FextAvg = [0, 0, 0, 0, 0, 0]
    self.FextMeas_cnt = 0
    self.FextMeas_done = False
    self.FextMeas_AvgNum = 20
    self.RpyAvg = [0.0, 0.0, 0.0]
    # open files to record the measured force 
    # and corresponding robot states
    
    # time_prefix = time.strftime('%Y%m%d%H%M%S',time.localtime())+'_'
    # log_robot_info_name = time_prefix + 'write_data.txt'
    # log_robot_info_name = time_prefix + 'write_data_identify.txt'
    # log_joint_pos_name  = time_prefix + 'position_data.txt'

    log_robot_info_name = 'write_data_identify.txt'
    log_joint_pos_name  = 'position_data.txt'

    self.log_states_file = path_prefix+'/log/'+log_robot_info_name
    self.log_joints_file = path_prefix+'/log/'+log_joint_pos_name
    self.log_file_init()
    self.path_prefix = path_prefix
  
  def joint_pos_init(self, init_joints_group_name):
    if not (init_joints_group_name == 'ready' or
            init_joints_group_name == 'ftcalib_jgroup') :
      rospy.loginfo("No moving! Valid commands include: ready / ftcalib_jgroup")
      return False
    else:
      self.commander.set_named_target(init_joints_group_name)
      try:
        self.commander.go()
      except:
        rospy.logerr('Fail to reach the target pose!')
        return False
      return True

  def running_data_acquisition(self, init_joints_group_name):
      self.data_ac_with_joints_cmd(init_joints_group_name)
    
  def data_ac_with_joints_cmd(self, init_joints_group_name):
    file_dir  = self.path_prefix + '/config/'
    file_name = 'joint_cmds.txt'
    joint_cmd_list = self.txt_data_2_lists(file_dir+file_name)
    for jcmd in joint_cmd_list:
      self.commander.set_joint_value_target(jcmd)
      self.commander.go()
      self.record_data()
    self.joint_pos_init(init_joints_group_name)

  # This function is used to load the data from txt to python
  # in list data type, where the first row is omitted
  # @input : path and name of the txt file
  # @return: lists of the data in the txt file
  def txt_data_2_lists(self, filename):
    with open(filename, 'r') as f:
        txt_list = f.readlines()[1::]
        data_list = []
        for txt_line in txt_list:
            txt_line = txt_line[0:-1]
            data_line = [float(entry) for entry in txt_line.split('\t')]
            data_list.append(data_line)
    return data_list

  def sub_F_ext_cb(self, msg):
    self.Fext = [msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
                 msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
    # Averaging the received data to suppress the random noise
    if self.FextMeas_done == False and self.FextMeas_cnt < self.FextMeas_AvgNum:
      for i, f in enumerate(self.Fext):
        self.FextAvg[i] = (self.FextMeas_cnt * self.FextAvg[i] + f)/(self.FextMeas_cnt+1)
      self.FextMeas_cnt += 1
                          

  def state_str(self):
    # averaging the measured forces
    self.FextAvg = [0, 0, 0, 0, 0, 0]
    self.FextMeas_cnt = 0
    self.FextMeas_done = False
    while(not self.FextMeas_done):
      if self.FextMeas_cnt >= self.FextMeas_AvgNum:
        self.FextMeas_done = True
    # averaging the measured rpys
    self.RpyAvg = [0.0, 0.0, 0.0]
    for i in range(self.FextMeas_AvgNum):
      self.rate2.sleep()
      rpy_tmp = self.commander.get_current_rpy(end_effector_link=self.ee_link)
      self.RpyAvg[0] += rpy_tmp[0]
      self.RpyAvg[1] += rpy_tmp[1]
      self.RpyAvg[2] += rpy_tmp[2]
    self.RpyAvg[0] /= self.FextMeas_AvgNum
    self.RpyAvg[1] /= self.FextMeas_AvgNum
    self.RpyAvg[2] /= self.FextMeas_AvgNum
    return  (str(self.RpyAvg)[1:-1]).replace(',','\t')  + '\t' + \
            (str(self.FextAvg)[1:-1]).replace(',','\t') + '\n'

  def joint_str(self):
    joint_v = self.commander.get_current_joint_values()
    return (str(joint_v)[1:-1]).replace(',','\t') + '\n' 

  def log_file_init(self):
    with open(self.log_states_file, 'w') as f:
      f.write('rpy.x'  + '\t' + 'rpy.y'  + '\t' + 'rpy.z'  + '\t' + 
              'Fext.x' + '\t' + 'Fext.y' + '\t' + 'Fext.z' + '\t' + 
              'Text.x' + '\t' + 'Text.y' + '\t' + 'Text.z' + '\n')
    with open(self.log_joints_file, 'w') as f:
      f.write('q1'+'\t' + 'q2'+'\t' + 'q3'+'\t' + \
              'q4'+'\t' + 'q5'+'\t' +'q6'+'\t' +'q7'+'\n')
  
  def record_data(self):
    self.rate.sleep()
    self.rate.sleep()
    self.rate.sleep()
    with open(self.log_states_file, 'a') as f:
      f.write(self.state_str())
    with open(self.log_joints_file, 'a') as f:
      f.write(self.joint_str())

def main():
  rospy.init_node('move_to_position', anonymous=True)
  rospy.wait_for_message('move_group/status', GoalStatusArray)
  try:
    joints_group_name = sys.argv[1] # input arguments should be either "ready" or "oa_ready"
  except Exception as e:
    rospy.loginfo(sys.argv)
    rospy.logerr(e)
  try:
    fcda = ForceCaliDataAcq(path_prefix = sys.argv[2],
                            end_effector_link = 'panda_link8')
    if fcda.joint_pos_init(joints_group_name):   # joints position initialization
      fcda.running_data_acquisition(joints_group_name)
  except rospy.ROSInterruptException:
    pass
  
  rospy.loginfo("Program done!") # data recording done!

if __name__ == '__main__':
  main()
