import numpy as np
from roboticstoolbox import mstraj, ctraj
from spatialmath import * 
from traj_gen import *
import os 
import matplotlib as plt
import ruckig 

dt = 0.004

tacc = 0.5

qdmax = 0.6 










current_dir = os.getcwd()
parent_dir = os.path.dirname(current_dir)

waypoints_path = 'car_output.txt'
cwaypoints_path = 'cwaypoints.txt'
waypoints = np.loadtxt(waypoints_path,dtype=float)
cwaypoints = np.loadtxt(cwaypoints_path,dtype=float)
if np.any(waypoints > 2 * np.pi):
    raise ValueError("Error: input waypoints are in degrees, radian expected")


T0 = SE3(cwaypoints[0,0],cwaypoints[0,1],cwaypoints[0,2]) * SE3.RPY([cwaypoints[0,3], cwaypoints[0,4], cwaypoints[0,5]], order='xyz')
T1 = SE3(cwaypoints[1,0],cwaypoints[1,1],cwaypoints[1,2]) * SE3.RPY([cwaypoints[1,3], cwaypoints[1,4], cwaypoints[1,5]], order='xyz')



traj_c = ctraj(T0=T0, T1=T1,t=20)

if __name__ == '__main__':





    car_space_traj_gen(cwaypoints_path, "traj_generated.txt", t=1000)





