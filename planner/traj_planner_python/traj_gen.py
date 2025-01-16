import numpy as np
from roboticstoolbox import mstraj, ctraj
from spatialmath import *
import os

def joint_space_traj_gen(waypoint_path, saved_traj_path, dt, tacc, qdmax):
    waypoints = np.loadtxt(waypoint_path, dtype=float)
    if np.any(waypoints > 2 * np.pi):
        raise ValueError("Error: input waypoints are in degrees, radian expected")
    
    n_rows = waypoints.shape[0]

    total_traj = np.empty([0,waypoints.shape[1]])
    for i in range(0, n_rows - 1):

        submatrix = waypoints[i:i+2, :]
        traj = mstraj(viapoints=submatrix, dt=dt, tacc=tacc, qdmax=qdmax)
        total_traj = np.vstack((total_traj,traj.q))


    np.savetxt(saved_traj_path, total_traj)
    return 

def car_space_traj_gen(waypoint_path, saved_traj_path, t = 100):
    def rotation_matrices_to_euler_angles(matrices, rotation_order='xyz'):
        euler_angles_list = []

        for matrix in matrices:
            if rotation_order == 'zyx':
                yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
                pitch = np.arctan2(-matrix[2, 0], np.sqrt(matrix[2, 1]**2 + matrix[2, 2]**2))
                roll = np.arctan2(matrix[2, 1], matrix[2, 2])
            elif rotation_order == 'xzx':
                roll = np.arctan2(matrix[0, 1], matrix[0, 2])
                pitch = np.arccos(matrix[0, 0])
                yaw = np.arctan2(matrix[1, 0], -matrix[2, 0])
            elif rotation_order == 'xyz':
                pitch = np.arcsin(-matrix[2, 0])
                roll = np.arctan2(matrix[2, 1] / np.cos(pitch), matrix[2, 2] / np.cos(pitch))
                yaw = np.arctan2(matrix[1, 0] / np.cos(pitch), matrix[0, 0] / np.cos(pitch))
            else:
                raise ValueError("Invalid rotation order. Supported orders: 'zyx', 'xzx', 'xyz'.")

            euler_angles_list.append((roll, pitch, yaw))

        return np.array(euler_angles_list)
    
    waypoints = np.loadtxt(waypoint_path, dtype=float)
    traj_c_total = np.empty([0,16])
    for i in range(waypoints.shape[0]-1):
        T0 = SE3(waypoints[i,0],waypoints[i,1],waypoints[i,2]) * SE3.RPY([waypoints[i,3], waypoints[i,4], waypoints[i,5]], order='xyz')
        T1 = SE3(waypoints[i+1,0],waypoints[i+1,1],waypoints[i+1,2]) * SE3.RPY([waypoints[i+1,3], waypoints[i+1,4], waypoints[i+1,5]], order='xyz')
        traj_c = ctraj(T0=T0, T1=T1, t=t)
        for i in range (traj_c.R.shape[0]):
            rotmat = traj_c.R[i]
            trans = traj_c.t[i]
            print(rotmat)
            print(trans)
            result_array = np.hstack((rotmat, trans.reshape((-1,1))))
            result_array = np.vstack((result_array, [0, 0, 0, 1]))
            flattened_array = result_array.flatten()
            traj_c_total = np.vstack((traj_c_total,flattened_array))
    np.savetxt(saved_traj_path, traj_c_total)
    np.savetxt('traj_generated.csv', traj_c_total, delimiter=',')
    return 
