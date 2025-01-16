import numpy as np
from roboticstoolbox import mstraj
dt = 0.004
tacc = 0.1
qdmax = 0.6 
waypoints_path = 'auto_cali_configuration_trajectory/traj.txt'
waypoints = np.loadtxt(waypoints_path,dtype=float)
if np.any(waypoints > 2 * np.pi):
    raise ValueError("Error: input waypoints are in degrees, radian expected")
traj = mstraj(viapoints=waypoints, dt=dt, tacc=tacc, qdmax=qdmax)
if __name__ == '__main__':
    print('pass')
    print(traj.q.shape)
    print(traj.q)
    np.savetxt('traj_generated.txt', traj.q)