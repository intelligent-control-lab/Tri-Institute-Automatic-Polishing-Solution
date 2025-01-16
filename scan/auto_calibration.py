
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
from icop_tracking import ICOP_EOAT_tracking
import sys
from scan_processing import savecirle, GP50_FK, sphereFit

EOA2laser_optimized_rotation_transition=np.array(
[[-0.99922368, -0.01255529, -0.03734215,  0.11405855],
 [ 0.03760676, -0.02154423, -0.9990603,  -0.04390233],
 [ 0.01173898, -0.99968907,  0.02199967, -0.24029726],
 [ 0.,          0.,          0.,          1.        ]]
)

"""
auto calibration parameters
"""
vmin = 160
vmax = 700
auto_movement_distance_from_center = 0.03
interpolation_number_per_scan_trajectory = 10
interpolation_number_per_move2start_trajectory = 20
ORIENTATION_COEFF = 1
height_idx_range = 3
radius_idx_range = 3
rotation_unit = -np.pi/20
initial_transition_guess = np.array([0.0852, -0.0559, -0.2523])


server_ip = '192.168.1.31'
server_port = 11000
message = "get_pos"
localIP = '0.0.0.0'

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('localhost',server_port)
client_socket.bind((localIP,server_port))

GP50_DH = np.array([[0, 0.540, 0.145, -np.pi/2],
                   [np.pi/2, 0, -0.87, np.pi],
                   [0, 0, -0.21, np.pi/2],
                   [0, -1.025, 0, -np.pi/2],
                   [0, 0, 0, np.pi/2],
                   [0, -0.175, 0, 0]])

reference_EOA2Laser = np.array([[1, 0, 0, 0.0852],
                                [0, 0, -1, -0.0559],
                                [0, 1, 0, -0.2523],
                                [0, 0, 0, 1]])

def get_robot_pos():
    """get the current robot pose

    Returns:
        ndarray: robot pose in radius
    """
    data, server_address = client_socket.recvfrom(24)
    joint_pos= []
    for i in range(0,24,4):
        temp = struct.unpack('f', data[i:i+4])
        joint_pos.append(temp[0])
    
    joint_pos_current_frame = np.array(joint_pos)
    print(joint_pos_current_frame)
    return joint_pos_current_frame
   
def auto_calibration_robot_executation_with_laser_recording():
    idx = 0
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    scan_trajectory_path = os.path.join(os.getcwd(), 'auto_cali_configuration_trajectory/')
    prefix_traj = 'traj'
    all_files = os.listdir(scan_trajectory_path)
    mathching_files_traj = [file for file in all_files if file.startswith(prefix_traj)]
    traj_totol_num = len(mathching_files_traj)
    for traj_idx in range(traj_totol_num):
        current_move2start_traj = np.loadtxt(scan_trajectory_path + f"move2start_traj{traj_idx}.txt")
        current_traj = np.loadtxt(scan_trajectory_path + f"traj{traj_idx}.txt")
        start_robot_pos = current_traj[0,:]
        end_robot_pos = current_traj[-1,:]
        start_robot_pos_move2start_traj = current_move2start_traj[0,:]
        end_robot_pos_move2start_traj = current_move2start_traj[-1,:]
        filename = f"auto_cali_configuration_trajectory/current_traj.txt"
        fmt = "%.5f"
        current_move2start_traj = current_move2start_traj.reshape(-1,6)
        np.savetxt(filename, current_move2start_traj, delimiter=' ', fmt=fmt)
        start_robot_pos = start_robot_pos/180 *np.pi
        end_robot_pos = end_robot_pos/180 *np.pi
        start_robot_pos_move2start = start_robot_pos_move2start_traj/180 *np.pi
        end_robot_pos_move2start = end_robot_pos_move2start_traj/180 *np.pi
        command = ['/home/icl/Documents/zhongqi/amr_pt2/build/laser_calibration']
        controller_subprocess = subprocess.Popen(command)
       
        while (1):
            current_robot_pos = get_robot_pos()
            print(traj_idx, ":", np.abs(current_robot_pos - end_robot_pos_move2start ).max())
            if np.abs(current_robot_pos - end_robot_pos_move2start).max() < 0.0001:
                controller_subprocess.kill()
                break
        print("starting laser")
        while(1):
            current_robot_pos = get_robot_pos()
            if np.abs(current_robot_pos - start_robot_pos).max() < 0.0001:
                print(f"{TextColor.GREEN}{TextColor.BOLD}start pose of trajectory {traj_idx} \
                    reached, start listening and execution{TextColor.RESET}")
                folder_path = 'collected_pyramid_data/' + str(traj_idx) + '/'
                command = ['python', 'laser_measure_motoplus_callable.py', folder_path]
                laser_listen_subprocess = subprocess.Popen(command)
                time.sleep(10)
                break
        np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
        command = ['/home/icl/Documents/zhongqi/amr_pt2/build/laser_calibration']
        controller_subprocess = subprocess.Popen(command)
        while (1):
            current_robot_pos = get_robot_pos()
            print(traj_idx, ":", np.abs(current_robot_pos - end_robot_pos).max())
            if np.abs(current_robot_pos - end_robot_pos).max() < 0.0001:
                controller_subprocess.kill()
                break
        while(1):
            current_robot_pos = get_robot_pos()
            if np.abs(current_robot_pos - end_robot_pos).max() < 0.0001:
                print(f"{TextColor.BLUE}{TextColor.BOLD}end pose of trajectory {traj_idx} \
                    reached, end listening and move to next!!{TextColor.RESET}")
                laser_listen_subprocess.terminate()
                break

def showcase_fitting_EOA2laser_transition():
    """
    showcase the fitted EOA2laser transition parameter performance
    ideally the laser beam can rotate perfectly around the pyramid tip 
    """
    movement_targets = np.zeros((0, 6))
    target_configuration = np.zeros((0,6))
    for rotation_angle in np.arange(np.pi, 0.2*np.pi, -np.pi/10):
        movement_target = np.append(sphr_cp_wld.squeeze() + np.array([0, 0, 0.25]), np.array([np.pi, 0, rotation_angle]))[np.newaxis,:]
        movement_targets = np.vstack((movement_targets, movement_target.reshape(1,-1)))
    start_ref_pose = np.array([0., 0., 0., 0.2618, 0.2269, 1.100])
    target_configuration = ICOP_EOAT_tracking(start_ref_pose, GP50_DH, movement_targets, EOA2laser_optimized_rotation_transition)
    target_configuration = (target_configuration) / np.pi *180
    filename = f"auto_cali_configuration_trajectory/current_traj.txt"
    filename_gazebo = "/home/icl/catkin_ws/src/Robot_Digital_Twin-main/joint_sequence_sim/joint_sim.txt"
    fmt = "%.5f"
    target_configuration = target_configuration.reshape(-1,6)
    np.savetxt(filename, target_configuration, delimiter=' ', fmt=fmt)
    np.savetxt(filename_gazebo, target_configuration, delimiter=' ', fmt=fmt)
    def confirm_trajectory():
        user_input = input(f"{TextColor.BLUE}Confirm the trajectory is examined in simulation (yes/no): {TextColor.RESET}").lower()
        if user_input == 'yes':
            print(f"{TextColor.GREEN}Continuing with the actual robot showcase execution...{TextColor.RESET}")
        elif user_input == 'no':
            print(f"{TextColor.RED}Please examine in Gazebo simulation before proceed.{TextColor.RESET}")
            sys.exit()
        else:
            print("Invalid input. Please enter 'yes' or 'no'.")
            confirm_trajectory()
    confirm_trajectory()
    
def xyzRxRyRz2transmat(xyzRxRyRz):
    '''xyzRxRyRz target transform into (4,4) transformation matrix'''
    transmat = np.identity(4, dtype=np.float32)
    xyzRxRyRz = xyzRxRyRz.reshape(6,)
    rotation = R.from_euler('xyz', xyzRxRyRz[3:], degrees=False)
    target_orientation = rotation.as_matrix()
    target_position = xyzRxRyRz[:3]
    transmat[:3,:3] = target_orientation
    transmat[:3,-1] = target_position
    return transmat


if __name__ == '__main__':
    showcase_fitting_EOA2laser_transition()
    