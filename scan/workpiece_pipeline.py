import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from tqdm import tqdm
from scipy.optimize import minimize
from scipy.optimize import least_squares
from laser_measure_motoplus import LaserSubscriber
from scipy.spatial.transform import Rotation as R
from detection_magic import TextColor, detect_pyramid_peak_fake_y
import time
import subprocess
import socket
import struct
import pickle
from icop_tracking import icop_start2goal, linear_interpolate
import sys
import ipdb

def excepthook(type, value, traceback):

    sys.__excepthook__(type, value, traceback)

    ipdb.post_mortem(traceback)


sys.excepthook = excepthook


server_ip = '192.168.1.31'
server_port = 11003
message = "get_pos"
localIP = '0.0.0.0'

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.bind((localIP,server_port))

optimized_EOA2Tool=np.array([[ 0.494599, -0.849144, -0.18527 , -0.0256  ],
                            [-0.805251, -0.527928,  0.269936,  0.056391],
                            [-0.327024,  0.01568 , -0.944886, -0.580479],
                            [ 0.      ,  0.      ,  0.      ,  1.      ]])

EOA2laser_SC3012_final=np.array([[ 0.494599, -0.849144, -0.18527 ,  0.034391],
                                [-0.805251, -0.527928,  0.269936, -0.039756],
                                [-0.327024,  0.01568 , -0.944886, -0.427299],
                                [ 0.      ,  0.      ,  0.      ,  1.      ]])

def get_robot_pos():

    data, server_address = client_socket.recvfrom(24)
    joint_pos= []
    for i in range(0,24,4):
        temp = struct.unpack('f', data[i:i+4])
        joint_pos.append(temp)
    
    joint_pos_current_frame = np.array(joint_pos)
    return joint_pos_current_frame

   
def exit_polish(save_path):
    '''call position control to exit the measurement trajectory'''
    print(f'{TextColor.BOLD}{TextColor.GREEN} Now Exit!!! {TextColor.RESET}')
    current_traj = np.loadtxt(f'{save_path}/rvrs_polish_traj.txt')


    cur_robo_pos = get_robot_pos()
    cur2traj = icop_start2goal(theta_start=cur_robo_pos.flatten(), theta_goal=current_traj[0,:], interpolate_step_size=0.0002, optimized_EOA2laser=optimized_EOA2Tool)
    current_traj = np.vstack((cur2traj, current_traj))
    

    filename = "Automatic-Polishing-Solution/RobotController/data/traj_generated.txt"
    fmt = "%.5f"
    np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
    end_pos = current_traj[-1, :]
    

    command = ['Automatic-Polishing-Solution/RobotController/build/PositinerController']
    print("Moving robot")
    controller_subprocess = subprocess.Popen(command)
    
    
    while (1):
        current_robot_pos = get_robot_pos()
        print(current_robot_pos)
        if np.abs(current_robot_pos.flatten() - end_pos.flatten()).max() < 0.0005:
            controller_subprocess.kill()
            break  
        
          
def polish_pos_ctrl():
    """ the first phase of polishing, 
        to move the current pose to follow the 
        position control part of polishing trajectory.
    """
    scan_trajectory_path = '/home/icl/polishingAutomatic-Polishing-Solution/scan/testplate_trajectory/' + WP_weld_name + '/polish_posctrl.txt'
    current_traj = np.loadtxt(scan_trajectory_path)
    cur_robo_pos = get_robot_pos()
    cur2traj = icop_start2goal(theta_start=cur_robo_pos.flatten(), theta_goal=current_traj[0,:], interpolate_step_size=0.0002, optimized_EOA2laser=optimized_EOA2Tool)
    current_traj = np.vstack((cur2traj, current_traj))
    
    filename = "Automatic-Polishing-Solution/RobotController/data/traj_generated.txt"
    fmt = "%.5f"
    np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
    end_pos = current_traj[-1, :]
    
    command = ['Automatic-Polishing-Solution/RobotController/build/PositinerController']
    print("Moving robot")
    controller_subprocess = subprocess.Popen(command)
    
    while (1):
        current_robot_pos = get_robot_pos()
        print(current_robot_pos)
        if np.abs(current_robot_pos.flatten() - end_pos.flatten()).max() < 0.0005:
            controller_subprocess.kill()
            break

def polish_frc_ctrl(save_path, jnt_itp=False, force_control=True):
    def equal_stable(a, b, epsilon=0.00001):
        if(abs(a-b)<=epsilon):
            return True
        return False
    
    def robotStable(j1, j2, j3, j4, j5, j6, joints):
        '''determine if robot pos is stabe at the current position'''
        if(equal_stable(j1, joints[0]) and equal_stable(j2, joints[1]) and equal_stable(j3, joints[2]) and 
           equal_stable(j4, joints[3]) and equal_stable(j5, joints[4]) and equal_stable(j6, joints[5])):
            return True
        return False
    

    file_path = f'{save_path}polish_frcctrl.txt'
    current_traj = np.loadtxt(file_path)
    file_path = f'{save_path}/polish_frcctrl_frctraj.txt'
    current_frc_traj = np.loadtxt(file_path)
    file_path = f'{save_path}/polish_frcctrl_cntct.txt'
    current_cntct_traj = np.loadtxt(file_path)
    cur_robo_pos = get_robot_pos()
    
    if not jnt_itp:
        mv_start2traj = icop_start2goal(theta_start=cur_robo_pos.flatten(), theta_goal=current_traj[0,:], interpolate_step_size=0.0005, optimized_EOA2laser=optimized_EOA2Tool)
    else:
        mv_start2traj = linear_interpolate( cur_robo_pos.flatten(),\
                                            current_traj[0,:],\
                                            interpolate_num=int(abs(cur_robo_pos.flatten()\
                                                                    - current_traj[0,:]).max()\
                                                                    /0.0004))
    mv_start2traj_frc = np.zeros((mv_start2traj.shape[0],6))
    mv_start2traj_cntct = np.tile(current_cntct_traj[0,:], (mv_start2traj.shape[0], 1))

    current_traj = np.vstack((mv_start2traj, current_traj))
    current_frc_traj = np.vstack((mv_start2traj_frc, current_frc_traj))
    current_cntct_traj = np.vstack((mv_start2traj_cntct, current_cntct_traj))
    assert current_frc_traj.shape[0] == current_traj.shape[0] and current_frc_traj.shape[0] == current_cntct_traj.shape[0], 'force trajectory and reference trajectory length should match'
    
    

    filename = "Automatic-Polishing-Solution/RobotController/data/traj_generated.txt"
    fmt = "%.5f"
    np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
    end_pos = current_traj[-1, :]

    filename = "Automatic-Polishing-Solution/RobotController/data/force_traj.txt"
    fmt = "%.5f"
    np.savetxt(filename, current_frc_traj, delimiter=' ', fmt=fmt)

    filename = "Automatic-Polishing-Solution/RobotController/data/contact_frame.txt"
    fmt = "%.5f"
    np.savetxt(filename, current_cntct_traj, delimiter=' ', fmt=fmt)
    

    if force_control:
        command = ['Automatic-Polishing-Solution/RobotController/build/ForceController']
    else:
        command = ['Automatic-Polishing-Solution/RobotController/build/PositinerController']
    print("Moving robot")
    controller_subprocess = subprocess.Popen(command)
    

    stable_count = 0
    stable = False
    pre_j1 = 0.
    pre_j2 = 0.
    pre_j3 = 0.
    pre_j4 = 0.
    pre_j5 = 0.
    pre_j6 = 0.
    

    
    while (1):
        current_robot_pos = get_robot_pos().flatten()
        

        if robotStable(pre_j1, pre_j2, pre_j3, pre_j4, pre_j5, pre_j6, current_robot_pos):
            stable_count += 1
            if stable_count > 200:

                stable = True
        else: 

            stable_count = 0
            stable = False
                

        pre_j1 = current_robot_pos[0]
        pre_j2 = current_robot_pos[1]
        pre_j3 = current_robot_pos[2]
        pre_j4 = current_robot_pos[3]
        pre_j5 = current_robot_pos[4]
        pre_j6 = current_robot_pos[5]    

        
        if np.abs(current_robot_pos.flatten() - end_pos.flatten()).max() < 0.001:
            controller_subprocess.kill()
            break
    
    
def run_demo_pipeline():
    iteration_idx = 0
    while(1):
        WP_weld_name = 'WP_DEMO'
        save_processed_result_path = f'workpiece_trajectory/{WP_weld_name}/'
        np.savetxt('Demo_state.txt', ["Robot Measuring"], fmt="%s")
        polish_frc_ctrl(save_path=save_processed_result_path, jnt_itp=True, force_control=False)
        np.savetxt('Demo_state.txt', ["Robot Exiting"], fmt="%s")
        exit_polish(save_path=save_processed_result_path)
        np.savetxt('Demo_state.txt', ["Robot Grinding"], fmt="%s")
        polish_frc_ctrl(save_path=save_processed_result_path, jnt_itp=True, force_control=True)
        np.savetxt('Demo_state.txt', ["Robot Exiting"], fmt="%s")
        exit_polish(save_path=save_processed_result_path)

def final_demo_pipeline():
    iteration_idx = 0 
    while(1):
        WP_weld_name = 'WP2_RHS'
        save_processed_result_path = f'workpiece_trajectory/{WP_weld_name}/'
        np.savetxt('Demo_state.txt', ["Robot Measuring"], fmt="%s")
        polish_frc_ctrl(save_path=save_processed_result_path, jnt_itp=True, force_control=False)
        np.savetxt('Demo_state.txt', ["Robot Exiting"], fmt="%s")
        exit_polish(save_path=save_processed_result_path)        
if __name__ == '__main__':
    final_demo_pipeline()
    