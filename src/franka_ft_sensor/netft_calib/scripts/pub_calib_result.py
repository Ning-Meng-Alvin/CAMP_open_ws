#!/usr/bin/env python
import numpy as np
import yaml
import rospy  
from geometry_msgs.msg import WrenchStamped
from moveit_commander import MoveGroupCommander
from tf import transformations as t

def trans_mtx(x, y, z):
    res =  np.eye(4)
    res[:3, 3] = np.array([x, y, z])
    return res 

def R_regression(gamma, beta, alpha):
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    cb = np.cos(beta)
    sb = np.sin(beta)
    sg = np.sin(gamma)
    cg = np.cos(gamma)
    # R_1_2 = [[ca*cb, ca*sb*sg-sa*cg, ca*sb*cg+sa*sg, 1, 0, 0],
    #          [sa*cb, sa*sb*sg+ca*cg, sa*sb*cg-ca*sg, 0, 1, 0],
    #          [-sb, cb*sg, cb*cg, 0, 0, 1]]
    # return R_1_2
    R_2_1 = [[ca*cb, sa*cb, -sb, 1, 0, 0],
             [ca*sb*sg-sa*cg, sa*sb*sg+ca*cg, cb*sg, 0, 1, 0],
             [ca*sb*cg+sa*sg, sa*sb*cg-ca*sg, cb*cg, 0, 0, 1]] 
    return R_2_1

def rotation_base_2_end(gamma, beta, alpha):
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    cb = np.cos(beta)
    sb = np.sin(beta)
    sg = np.sin(gamma)
    cg = np.cos(gamma)
    R_ee_b = [[ca*cb, sa*cb, -sb],
              [ca*sb*sg-sa*cg, sa*sb*sg+ca*cg, cb*sg],
              [ca*sb*cg+sa*sg, sa*sb*cg-ca*sg, cb*cg]] 
    return R_ee_b


def calibration_simple(measured_ft, curr_rpy, identify_params):
    [Fx, Fy, Fz, Mx, My, Mz] = measured_ft
    [Fx0, Fy0, Fz0, Mx0, My0, Mz0, 
     Lx, Ly, Lz, cmx, cmy, cmz] = identify_params
    
    # get the gravity decomposition according to the orientation
    gamma, beta, alpha = curr_rpy
    R_ee_b = rotation_base_2_end(gamma, beta, alpha)
    G_arr = np.dot(np.array(R_ee_b), np.array([[Lx], [Ly], [Lz]]))
    [Gx, Gy, Gz] = G_arr.transpose()[0].tolist()
    
    Mgx = Gz*cmy - Gy*cmz
    Mgy = Gx*cmz - Gz*cmx
    Mgz = Gy*cmx - Gx*cmy

    Fex = Fx - Fx0 - Gx
    Fey = Fy - Fy0 - Gy
    Fez = Fz - Fz0 - Gz

    Mex = Mx - Mx0 - Mgx
    Mey = My - My0 - Mgy
    Mez = Mz - Mz0 - Mgz
    return [Fex, Fey, Fez, Mex, Mey, Mez]

def sub_F_ext_cb(msg):
    global Fext
    Fext = [msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

def ft_sensor_frame(rpy):  # TODO 
    # the rpy comes from panda_link8 at the flange
    # here we have to express them w.r.t. FT frame
    tf_tmp = t.euler_matrix(*rpy)
    Rz = t.rotation_matrix(-105.0*np.pi/180.0, [0, 0, 1])
    Tz = trans_mtx(0.0, 0.0, 0.055)    
    tf_tmp = np.dot(tf_tmp, np.dot(Tz, Rz))
    rpy = t.euler_from_matrix(tf_tmp)
    return rpy

def set_calb_mtx_into_yaml(file_path, identify_params):
    d = dict()
    d["netft_calib_param"] = dict() 
    [Fx0, Fy0, Fz0, Mx0, My0, Mz0, Lx, Ly, Lz, mcx, mcy, mcz] = identify_params
    with open(file_path, 'w') as f:
        d["netft_calib_param"]["Fx0"] = Fx0
        d["netft_calib_param"]["Fy0"] = Fy0
        d["netft_calib_param"]["Fz0"] = Fz0
        d["netft_calib_param"]["Mx0"] = Mx0
        d["netft_calib_param"]["My0"] = My0
        d["netft_calib_param"]["Mz0"] = Mz0
        d["netft_calib_param"]["Lx"] = Lx
        d["netft_calib_param"]["Ly"] = Ly
        d["netft_calib_param"]["Lz"] = Lz
        d["netft_calib_param"]["mcx"] = mcx
        d["netft_calib_param"]["mcy"] = mcy
        d["netft_calib_param"]["mcz"] = mcz
        yaml.dump(d, f)

Fext = [0,0,0,0,0,0]
if __name__ == '__main__':
    # get params
    netft_calib_param = rospy.get_param("/netft_calib_param")
    Fx0 = netft_calib_param["Fx0"]
    Fy0 = netft_calib_param["Fy0"]
    Fz0 = netft_calib_param["Fz0"]
    Lx = netft_calib_param["Lx"]
    Ly = netft_calib_param["Ly"]
    Lz = netft_calib_param["Lz"]
    Mx0 = netft_calib_param["Mx0"]
    My0 = netft_calib_param["My0"]
    Mz0 = netft_calib_param["Mz0"]
    mcx = netft_calib_param["mcx"]
    mcy = netft_calib_param["mcy"]
    mcz =  netft_calib_param["mcz"]
    identify_params = [Fx0, Fy0, Fz0, Mx0, My0, Mz0, Lx, Ly, Lz, mcx, mcy, mcz]
    print(identify_params)

    # calibration verification
    rospy.init_node("FTCalibration", anonymous=True)
    rate = rospy.Rate(400)

    commander = MoveGroupCommander('panda_arm')
    _sub_fext = rospy.Subscriber("/netft_data", 
                                  WrenchStamped, sub_F_ext_cb,
                                  queue_size=1, tcp_nodelay=True)
    _pub_cali_fext = rospy.Publisher(
        "/franka_state_controller/Cali_F_ext",
        WrenchStamped, queue_size=1)

    while not rospy.is_shutdown():
        rpy_v = commander.get_current_rpy(end_effector_link='panda_link8')
        rpy_v = ft_sensor_frame(rpy_v)
        measured_ft = [Fext[0], Fext[1], Fext[2], Fext[3], Fext[4], Fext[5]]
        [Fex, Fey, Fez, Mex, Mey, Mez] = calibration_simple(measured_ft, rpy_v, identify_params)
        
        # twist transformation F_a_b
        # for Acuity Echo Probe
        # F = np.array([[ 1.50383733e-01, -8.52868532e-01,  5.00000000e-01,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
        #               [ 9.84807753e-01,  1.73648178e-01,  0.00000000e+00,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
        #               [-8.68240888e-02,  4.92403877e-01,  8.66025404e-01,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
        #               [ 1.15137252e-01,  2.03018040e-02,  0.00000000e+00,  1.50383733e-01,  -8.52868532e-01,  5.00000000e-01],
        #               [-2.34425040e-02,  1.32949047e-01,  2.08166817e-17,  9.84807753e-01,   1.73648178e-01,  0.00000000e+00],
        #               [-6.64745233e-02, -1.17212520e-02,  0.00000000e+00, -8.68240888e-02,   4.92403877e-01,  8.66025404e-01]])
        
        # for vascular probe holder
        # F = np.array([[ 7.07106781e-01,  0.00000000e+00, -7.07106781e-01,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
        #               [ 0.00000000e+00,  1.00000000e+00, -5.74693726e-17,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
        #               [ 7.07106781e-01,  0.00000000e+00,  7.07106781e-01,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
        #               [ 2.84458882e-18,  1.25040000e-01, -4.34138154e-18,  7.07106781e-01,   0.00000000e+00, -7.07106781e-01],
        #               [-1.26359982e-01,  0.00000000e+00,  5.04732820e-02,  0.00000000e+00,   1.00000000e+00, -5.74693726e-17],
        #               [-2.84458882e-18,  5.36600000e-02, -2.39217718e-19,  7.07106781e-01,   0.00000000e+00,  7.07106781e-01]])
        
        # for Juniper Linear probe
        # Ftf = np.array([[ 8.32667268e-17, -1.00000000e+00,  0.00000000e+00,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],  # length from sensor to probe: 0.21
        #                 [ 1.00000000e+00,  8.32667268e-17,  0.00000000e+00,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
        #                 [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
        #                 [ 2.10000000e-01,  1.74860126e-17,  0.00000000e+00,  8.32667268e-17,  -1.00000000e+00,  0.00000000e+00],
        #                 [-1.74860126e-17,  2.10000000e-01,  0.00000000e+00,  1.00000000e+00,   8.32667268e-17,  0.00000000e+00],
        #                 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,   0.00000000e+00,  1.00000000e+00]])
        
        Ftf = np.array([[ 7.10137241e-17, -1.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00],  # length from sensor to probe: 0.185
                        [ 1.00000000e+00,  9.07576257e-17,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
                        [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
                        [ 1.85000000e-01,  1.67901608e-17,  0.00000000e+00,  7.10137241e-17, -1.00000000e+00,  0.00000000e+00],
                        [-1.31375390e-17,  1.85000000e-01,  0.00000000e+00,  1.00000000e+00,  9.07576257e-17,  0.00000000e+00],
                        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        
        fmeas = np.array([[Fex], [Fey], [Fez], [Mex], [Mey], [Mez]])
        Fcali = np.dot(Ftf, fmeas)

        # publish data
        cali_fext = WrenchStamped()
        cali_fext.wrench.force.x = Fcali[0]
        cali_fext.wrench.force.y = Fcali[1]
        cali_fext.wrench.force.z = Fcali[2]
        cali_fext.wrench.torque.x = Fcali[3]
        cali_fext.wrench.torque.y = Fcali[4]
        cali_fext.wrench.torque.z = Fcali[5]
        _pub_cali_fext.publish(cali_fext)
        rate.sleep()

