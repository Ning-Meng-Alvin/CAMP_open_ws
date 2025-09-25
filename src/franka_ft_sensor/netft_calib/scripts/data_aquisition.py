#!/usr/bin/env python3
# import rospy
# from moveit_commander import MoveGroupCommander
# from actionlib_msgs.msg import GoalStatusArray

# import sys
# import time
# from geometry_msgs.msg import WrenchStamped

# class ForceCaliDataAcq(object):
#   def __init__(self, path_prefix, end_effector_link):
#     rospy.loginfo("Data acquisition for force calibration")
#     # global variables
#     self.Fext = []
#     self.rate  = rospy.Rate(1)  # Hz
#     self.rate2 = rospy.Rate(50) 
#     self.ee_link = end_effector_link
#     self.commander = MoveGroupCommander('Panda_arm')
#     _sub_fext = rospy.Subscriber("/netft_data", 
#                                   WrenchStamped, self.sub_F_ext_cb,
#                                   queue_size=1, tcp_nodelay=True)
#     self.FextAvg = [0, 0, 0, 0, 0, 0]
#     self.FextMeas_cnt = 0
#     self.FextMeas_done = False
#     self.FextMeas_AvgNum = 20
#     self.RpyAvg = [0.0, 0.0, 0.0]
#     # open files to record the measured force 
#     # and corresponding robot states
    
#     # time_prefix = time.strftime('%Y%m%d%H%M%S',time.localtime())+'_'
#     # log_robot_info_name = time_prefix + 'write_data.txt'
#     # log_robot_info_name = time_prefix + 'write_data_identify.txt'
#     # log_joint_pos_name  = time_prefix + 'position_data.txt'

#     log_robot_info_name = 'write_data_identify.txt'
#     log_joint_pos_name  = 'position_data.txt'

#     self.log_states_file = path_prefix+'/log/'+log_robot_info_name
#     self.log_joints_file = path_prefix+'/log/'+log_joint_pos_name
#     self.log_file_init()
#     self.path_prefix = path_prefix
  
#   def joint_pos_init(self, init_joints_group_name):
#     if not (init_joints_group_name == 'ready' or
#             init_joints_group_name == 'ftcalib_jgroup') :
#       rospy.loginfo("No moving! Valid commands include: ready / ftcalib_jgroup")
#       return False
#     else:
#       self.commander.set_named_target(init_joints_group_name)
#       try:
#         self.commander.go()
#       except:
#         rospy.logerr('Fail to reach the target pose!')
#         return False
#       return True

#   def running_data_acquisition(self, init_joints_group_name):
#       self.data_ac_with_joints_cmd(init_joints_group_name)
    
#   def data_ac_with_joints_cmd(self, init_joints_group_name):
#     file_dir  = self.path_prefix + '/config/'
#     file_name = 'joint_cmds.txt'
#     joint_cmd_list = self.txt_data_2_lists(file_dir+file_name)
#     for jcmd in joint_cmd_list:
#       self.commander.set_joint_value_target(jcmd)
#       self.commander.go()
#       self.record_data()
#     self.joint_pos_init(init_joints_group_name)

#   # This function is used to load the data from txt to python
#   # in list data type, where the first row is omitted
#   # @input : path and name of the txt file
#   # @return: lists of the data in the txt file
#   def txt_data_2_lists(self, filename):
#     with open(filename, 'r') as f:
#         txt_list = f.readlines()[1::]
#         data_list = []
#         for txt_line in txt_list:
#             txt_line = txt_line[0:-1]
#             data_line = [float(entry) for entry in txt_line.split('\t')]
#             data_list.append(data_line)
#     return data_list

#   def sub_F_ext_cb(self, msg:WrenchStamped):
#     self.Fext = [msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
#                  msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
#     # Averaging the received data to suppress the random noise
#     if self.FextMeas_done == False and self.FextMeas_cnt < self.FextMeas_AvgNum:
#       for i, f in enumerate(self.Fext):
#         self.FextAvg[i] = (self.FextMeas_cnt * self.FextAvg[i] + f)/(self.FextMeas_cnt+1)
#       self.FextMeas_cnt += 1
                          

#   def state_str(self):
#     # averaging the measured forces
#     self.FextAvg = [0, 0, 0, 0, 0, 0]
#     self.FextMeas_cnt = 0
#     self.FextMeas_done = False
#     while(not self.FextMeas_done):
#       if self.FextMeas_cnt >= self.FextMeas_AvgNum:
#         self.FextMeas_done = True
#     # averaging the measured rpys
#     self.RpyAvg = [0.0, 0.0, 0.0]
#     for i in range(self.FextMeas_AvgNum):
#       self.rate2.sleep()
#       rpy_tmp = self.commander.get_current_rpy(end_effector_link=self.ee_link)
#       self.RpyAvg[0] += rpy_tmp[0]
#       self.RpyAvg[1] += rpy_tmp[1]
#       self.RpyAvg[2] += rpy_tmp[2]
#     self.RpyAvg[0] /= self.FextMeas_AvgNum
#     self.RpyAvg[1] /= self.FextMeas_AvgNum
#     self.RpyAvg[2] /= self.FextMeas_AvgNum
#     return  (str(self.RpyAvg)[1:-1]).replace(',','\t')  + '\t' + \
#             (str(self.FextAvg)[1:-1]).replace(',','\t') + '\n'

#   def joint_str(self):
#     joint_v = self.commander.get_current_joint_values()
#     return (str(joint_v)[1:-1]).replace(',','\t') + '\n' 

#   def log_file_init(self):
#     with open(self.log_states_file, 'w') as f:
#       f.write('rpy.x'  + '\t' + 'rpy.y'  + '\t' + 'rpy.z'  + '\t' + 
#               'Fext.x' + '\t' + 'Fext.y' + '\t' + 'Fext.z' + '\t' + 
#               'Text.x' + '\t' + 'Text.y' + '\t' + 'Text.z' + '\n')
#     with open(self.log_joints_file, 'w') as f:
#       f.write('q1'+'\t' + 'q2'+'\t' + 'q3'+'\t' + \
#               'q4'+'\t' + 'q5'+'\t' +'q6'+'\t' +'q7'+'\n')
  
#   def record_data(self):
#     self.rate.sleep()
#     self.rate.sleep()
#     self.rate.sleep()
#     with open(self.log_states_file, 'a') as f:
#       f.write(self.state_str())
#     with open(self.log_joints_file, 'a') as f:
#       f.write(self.joint_str())

# def main():
#   rospy.init_node('move_to_position', anonymous=True)
#   rospy.wait_for_message('move_group/status', GoalStatusArray)
#   try:
#     joints_group_name = sys.argv[1] # input arguments should be either "ready" or "oa_ready"
#   except Exception as e:
#     rospy.loginfo(sys.argv)
#     rospy.logerr(e)
#   try:
#     fcda = ForceCaliDataAcq(path_prefix = sys.argv[2],
#                             end_effector_link = 'panda_link8')
#     if fcda.joint_pos_init(joints_group_name):   # joints position initialization
#       fcda.running_data_acquisition(joints_group_name)
#   except rospy.ROSInterruptException:
#     pass
  
#   rospy.loginfo("Program done!")


# if __name__ == '__main__':
#   main()





import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import WrenchStamped
import sys
import time

class ForceCaliDataAcq:
    def __init__(self, path_prefix, end_effector_link):
        rospy.loginfo("Force calibration data acquisition started")

        self.path_prefix = path_prefix
        self.ee_link = end_effector_link
        self.Fext = []
        self.FextAvg = [0.0] * 6
        self.FextMeasCnt = 0
        self.FextMeasDone = False
        self.FextMeasAvgNum = 20
        self.RpyAvg = [0.0, 0.0, 0.0]

        self.rate1hz = rospy.Rate(1)
        self.rate50hz = rospy.Rate(50)

        self.commander = MoveGroupCommander('Panda_arm')

        # Subscriber
        self._sub_fext = rospy.Subscriber("/netft_data", WrenchStamped,
                                          self.sub_F_ext_cb, queue_size=1, tcp_nodelay=True)

        # Log files
        self.log_states_file = f"{path_prefix}/log/write_data_identify.txt"
        self.log_joints_file = f"{path_prefix}/log/position_data.txt"
        self.init_log_files()

    # Initialize joint positions
    def joint_pos_init(self, group_name):
        if group_name not in ['ready', 'ftcalib_jgroup']:
            rospy.loginfo("No moving! Valid names: ready / ftcalib_jgroup")
            return False
        self.commander.set_named_target(group_name)
        try:
            self.commander.go(wait=True)
        except Exception as e:
            rospy.logerr(f"Failed to reach target: {e}")
            return False
        return True

    # Run the full acquisition
    def run_data_acquisition(self, init_group_name):
        self.acquisition_with_joint_commands(init_group_name)

    # Acquire data following joint commands from txt
    def acquisition_with_joint_commands(self, init_group_name):
        joint_cmds_file = f"{self.path_prefix}/config/joint_cmds.txt"
        joint_cmd_list = self.txt_to_list(joint_cmds_file)
        for cmd in joint_cmd_list:
            self.commander.set_joint_value_target(cmd)
            # Non-blocking go with timeout
            start_time = time.time()
            success = False
            while not success and (time.time() - start_time < 5.0):
                success = self.commander.go(wait=False)
                rospy.sleep(0.01)
            if not success:
                rospy.logwarn("Joint command execution timeout")
            self.record_data()
        self.joint_pos_init(init_group_name)

    # Load txt to list (omit header)
    def txt_to_list(self, filename):
        with open(filename, 'r') as f:
            lines = f.readlines()[1:]
        data_list = []
        for line in lines:
            line = line.strip()
            data_list.append([float(x) for x in line.split('\t')])
        return data_list

    # Force sensor callback
    def sub_F_ext_cb(self, msg: WrenchStamped):
        self.Fext = [
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
        ]
        if not self.FextMeasDone and self.FextMeasCnt < self.FextMeasAvgNum:
            for i, f in enumerate(self.Fext):
                self.FextAvg[i] = (self.FextMeasCnt * self.FextAvg[i] + f) / (self.FextMeasCnt + 1)
            self.FextMeasCnt += 1

    # Generate state string
    def state_str(self):
        # Reset averaging
        self.FextAvg = [0.0] * 6
        self.FextMeasCnt = 0
        self.FextMeasDone = False

        # Wait for force averaging with timeout
        start_time = time.time()
        while not self.FextMeasDone and (time.time() - start_time < 5.0):
            if self.FextMeasCnt >= self.FextMeasAvgNum:
                self.FextMeasDone = True
            rospy.sleep(0.01)
        if not self.FextMeasDone:
            rospy.logwarn("Force averaging timeout")
            self.FextMeasDone = True

        # Average RPY
        self.RpyAvg = [0.0, 0.0, 0.0]
        for _ in range(self.FextMeasAvgNum):
            self.rate50hz.sleep()
            rpy_tmp = self.commander.get_current_rpy(end_effector_link=self.ee_link)
            for i in range(3):
                self.RpyAvg[i] += rpy_tmp[i]
        self.RpyAvg = [x / self.FextMeasAvgNum for x in self.RpyAvg]

        rpy_str = '\t'.join(f"{x:.6f}" for x in self.RpyAvg)
        fext_str = '\t'.join(f"{x:.6f}" for x in self.FextAvg)
        return f"{rpy_str}\t{fext_str}\n"

    # Generate joint string
    def joint_str(self):
        joint_vals = self.commander.get_current_joint_values()
        return '\t'.join(f"{x:.6f}" for x in joint_vals) + '\n'

    # Initialize log files
    def init_log_files(self):
        with open(self.log_states_file, 'w') as f:
            f.write('rpy.x\trpy.y\trpy.z\tFext.x\tFext.y\tFext.z\tText.x\tText.y\tText.z\n')
        with open(self.log_joints_file, 'w') as f:
            f.write('q1\tq2\tq3\tq4\tq5\tq6\tq7\n')

    # Record one step of data
    def record_data(self):
        self.rate1hz.sleep()
        self.rate1hz.sleep()
        self.rate1hz.sleep()
        with open(self.log_states_file, 'a') as f:
            f.write(self.state_str())
        with open(self.log_joints_file, 'a') as f:
            f.write(self.joint_str())


def main():
    rospy.init_node('move_to_position', anonymous=True)  # Node name unchanged
    rospy.wait_for_message('move_group/status', GoalStatusArray)

    try:
        joint_group_name = sys.argv[1]  # "ready" or "ftcalib_jgroup"
        path_prefix = sys.argv[2]
    except Exception as e:
        rospy.logerr(f"Invalid arguments: {sys.argv}, {e}")
        sys.exit(1)

    try:
        fcda = ForceCaliDataAcq(path_prefix=path_prefix, end_effector_link='panda_link8')
        if fcda.joint_pos_init(joint_group_name):
            fcda.run_data_acquisition(joint_group_name)
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt, exiting")
    finally:
        rospy.loginfo("Program done!")
        rospy.signal_shutdown("Finished")
        sys.exit(0)


if __name__ == '__main__':
    main()
