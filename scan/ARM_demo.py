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
from icop_tracking import ICOP_EOAT_tracking, transmat2xyzRxRyRz, forward_kinematics
import sys
from scan_processing import savecirle, GP50_FK, sphereFit
path = "Demo_state.txt"

def go_home(t = 15):
    command = ['/home/icl/Documents/zhongqi/amr_pt2/build/go_home']
    controller_subprocess = subprocess.Popen(command)
    print("Robot Going to Home Position")
    time.sleep(t)
    controller_subprocess.kill()

def go_measure():
    command = ['/home/icl/Documents/zhongqi/amr_pt2/build/go_measure']
    controller_subprocess = subprocess.Popen(command)
    print("Robot Going to Measurement Starting Position")
    time.sleep(15)
    controller_subprocess.kill()

def measure():
    command = ['/home/icl/Documents/zhongqi/amr_pt2/build/measure']
    controller_subprocess = subprocess.Popen(command)
    print("Robot start Measuring")
    time.sleep(10)
    controller_subprocess.kill()
    
def go_polishing(t = 15):
    command = ['/home/icl/Documents/zhongqi/amr_pt2/build/go_polishing']
    controller_subprocess = subprocess.Popen(command)
    print("Robot Going to Polishing Starting Position")
    time.sleep(t)
    controller_subprocess.kill()

def polishing():
    command = ['/home/icl/Documents/zhongqi/amr_pt2/build/polishing']
    controller_subprocess = subprocess.Popen(command)
    print("Robot start Position")
    time.sleep(60)
    controller_subprocess.kill()

def polishing_position():
    command = ['/home/icl/Documents/zhongqi/amr_pt2/build/polishing_position']
    controller_subprocess = subprocess.Popen(command)
    print("Robot start Position")
    time.sleep(20)
    controller_subprocess.kill()

def laser_scan():
    scan_trajectory_path = '/home/icl/polishing/Automatic-Polishing-Solution/scan/auto_cali_configuration_trajectory/'
    time_list = [15,12,8,15,10,10,15,10,10]
    traj_num = 9
    for traj_idx in range(traj_num):
        np.savetxt(path, [f"Laser Scanning {traj_idx + 1} / {traj_num}"], fmt="%s")
        current_traj = np.loadtxt(scan_trajectory_path + f"traj{traj_idx}.txt")
        filename = f"auto_cali_configuration_trajectory/current_traj.txt"
        fmt = "%.5f"
        current_traj = current_traj.reshape(-1,6)
        if traj_idx == 0:
            current_traj_1 = current_traj[:145,:]
            current_traj_2 = current_traj[145:,:]
            current_traj_1 = current_traj_1[::5,:]

            current_traj = current_traj_2
        end_robot_pos = current_traj[-1,:]
        end_robot_pos = end_robot_pos/180 *np.pi
        np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
        command = ['/home/icl/Documents/zhongqi/amr_pt2/build/laser_calibration']
        controller_subprocess = subprocess.Popen(command)
        time.sleep(time_list[traj_idx])
        controller_subprocess.kill()

def laser_rotate():
    scan_trajectory_path = '/home/icl/polishing/Automatic-Polishing-Solution/scan/rotation_demo.txt'
    current_traj = np.loadtxt(scan_trajectory_path)
    filename = f"auto_cali_configuration_trajectory/current_traj.txt"
    fmt = "%.5f"
    current_traj = current_traj.reshape(-1,6)
    current_traj = current_traj[155:,:]
    np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
    command = ['/home/icl/Documents/zhongqi/amr_pt2/build/laser_calibration']
    controller_subprocess = subprocess.Popen(command)
    time.sleep(45)
    controller_subprocess.kill()

def polish_workpiece():
    scan_trajectory_path = '/home/icl/polishing/Automatic-Polishing-Solution/scan/auto_cali_configuration_trajectory/current_traj_itp.txt'
    current_traj = np.loadtxt(scan_trajectory_path)
    filename = f"auto_cali_configuration_trajectory/current_traj.txt"
    fmt = "%.5f"
    current_traj = current_traj.reshape(-1,6)
    np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
    command = ['/home/icl/Documents/zhongqi/amr_pt2/build/laser_calibration']
    controller_subprocess = subprocess.Popen(command)
    time.sleep(1000)
    controller_subprocess.kill()

def measure_polishing_demo():
    np.savetxt(path, ["Robot Going Home"], fmt="%s")
    go_home(t = 2)
    polish_workpiece()
    
def confirm_trajectory():
    user_input = input(f"{TextColor.BLUE}Confirm the trajectory is examined in simulation (yes/no): {TextColor.RESET}").lower()

    if user_input == 'yes':
        print(f"{TextColor.GREEN}Continuing with the actual robot showcase execution...{TextColor.RESET}")

    elif user_input == 'no':
        print(f"{TextColor.RED}Please examine in Gazebo simulation before proceed.{TextColor.RESET}")
        sys.exit()
    else:
        print("Invalid input. Please enter 'yes' or 'no'.")
 
if __name__ == '__main__':
    measure_polishing_demo()





    