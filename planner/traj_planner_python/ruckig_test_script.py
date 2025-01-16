import numpy as np
from roboticstoolbox import mstraj, ctraj
from spatialmath import * 
from traj_gen import *
import os 
import matplotlib as plt
from ruckig_traj import *
from ruckig import InputParameter, Ruckig, Trajectory, Result

dt = 0.004
tacc = 0.1
qdmax = 0.6 
current_dir = os.getcwd()
parent_dir = os.path.dirname(current_dir)
waypoints_path = parent_dir + '/scan/auto_cali_configuration_trajectory/current_traj.txt'
cwaypoints_path = 'cwaypoints.txt'
waypoints = np.loadtxt(waypoints_path,dtype=float)
cwaypoints = np.loadtxt(cwaypoints_path,dtype=float)

if __name__ == '__main__':
    joint_space_traj_gen(waypoint_path = 'car_output.txt', saved_traj_path = 'angle_interp.txt', dt=0.004, tacc = tacc, qdmax = qdmax)



 



 



 



 



 




 

 

 


 

 

