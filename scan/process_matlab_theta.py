import numpy as np
import os

current_traj = np.loadtxt("/home/icl/polishing/Automatic-Polishing-Solution/scan/data/MATLAB_result/WP2_LHS/left_polish.txt", delimiter=",")
filename = f"/home/icl/polishing/Automatic-Polishing-Solution/scan/workpiece_trajectory/WP2_LHS/ref_polish_traj.txt"
if os.path.exists(filename):
    os.remove(filename)
fmt = "%.5f"
current_traj = current_traj.reshape(-1,6)
current_traj[:,2] *= -1
current_traj[:,3] *= -1
current_traj[:,4] *= -1
current_traj[:,5] *= -1
np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
current_traj = np.loadtxt("/home/icl/polishing/Automatic-Polishing-Solution/scan/data/MATLAB_result/WP2_LHS/left_polish_mask.txt", delimiter=" ")
filename = f"/home/icl/polishing/Automatic-Polishing-Solution/scan/workpiece_trajectory/WP2_LHS/ref_polish_traj_mask.txt"
if os.path.exists(filename):
    os.remove(filename)
fmt = "%d"
current_traj = current_traj.reshape(-1,1)
np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)
current_traj = np.loadtxt("/home/icl/polishing/Automatic-Polishing-Solution/scan/data/MATLAB_result/WP2_LHS/left_polish_frame.txt", delimiter=",")
filename = f"/home/icl/polishing/Automatic-Polishing-Solution/scan/workpiece_trajectory/WP2_LHS/ref_polish_traj_frame.txt"
if os.path.exists(filename):
    os.remove(filename)
fmt = "%5f"
current_traj = current_traj.reshape(-1,9)
np.savetxt(filename, current_traj, delimiter=' ', fmt=fmt)