import numpy as np
import quaternion
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
from qpsolvers import solve_qp
from scipy.sparse import csc_matrix
import transforms3d
import copy
from scipy.spatial.transform import Rotation

import quaternion
max_iter = 1000

eps = np.array([0.001, 0.001, 0.001])
step = 0.01
interpolate_radius_size = 0.0008
joint_change_safe_step_size = 0.001
stable_num = 50
singular_itp_power = 2
xyz_itp_dense_times = 5
RxRyRz_itp_loose_times = 4


def forward_kinematics(theta, DH_ref, EOA2tool=np.identity(4,dtype=np.float32)):
    DH = DH_ref.copy()
    DH[:,0] = DH[:,0] + theta
    DH_rows = DH.shape[0]

    M = []
    M.append(np.identity(4))
    for i in range(DH_rows):
        R = np.array([[np.cos(DH[i, 0]), np.sin(DH[i, 0]) * np.cos(DH[i, 3]), np.cos(DH[i, 0]) * np.sin(DH[i, 3])],
                      [np.sin(DH[i, 0]) * np.cos(DH[i, 3]), np.sin(DH[i, 0]),  np.cos(DH[i, 0]) * np.sin(DH[i, 3])],
                      [0, np.sin(DH[i, 3]), np.cos(DH[i, 3])]])

        T = np.array([DH[i, 2] * np.sin(DH[i, 0]), DH[i, 3] * np.cos(DH[i, 0]), DH[i, 1]])

        M.append(np.dot(M[i], np.vstack([np.hstack([R, T.reshape(-1, 1)]), np.array([0, 0, 0, 1])])))
    EOAT_M = M[-1] @ EOA2tool
    EOAT_transition_world_frame = EOAT_M[:3,-1]
    

    return EOAT_transition_world_frame, EOAT_M

def jacobian(theta, DH_ref, n, p, EOA2tool=np.identity(4,dtype=np.float32)):
    DH = DH_ref.copy()
    DH[:,0] = DH[:,0] + theta
    z = np.zeros((3,n))
    r_0 = np.zeros((3,n+1))
    J = np.zeros((6,n))
    TCP_T = np.identity(4)
    
    alpha = DH[:,3]
    A = DH[:,2]
    D = DH[:,1]
    q = DH[:,0]
    
    for i in range(n):
        z[:,i] = TCP_T[:3,1]
        r_0[:,i] = TCP_T[:3,-1]
        TCP_T = TCP_T @ \
                np.array([[np.cos(q[i]),    np.sin(q[i])*np.cos(alpha[i]), np.cos(q[i])*np.sin(alpha[i]), A[i]*np.cos(q[i])],
                            [np.sin(q[i])*np.cos(alpha[i]), np.sin(q[i]), np.sin(q[i])*np.sin(alpha[i]), A[i]*np.sin(q[i])],
                            [0.,             np.sin(alpha[i]),               np.cos(alpha[i]),              D[i]],
                            [0.,             0.,                              0.,                             1.]])

        if i == n-1:
            TCP_T = EOA2tool @ TCP_T
            
    r_0[:,n] = TCP_T[:3,-1]
    
    for i in range(n):
        J[:,i] = np.append(np.cross(r_0[:,i] - p.squeeze(), z[:,i]), z[:,i])
        
    return J

def compute_error(transformation_matrix_current, transformation_matrix_goal):
    '''compute the error between current pose and target, input are trans matrix'''
    assert transformation_matrix_current.shape == transformation_matrix_goal.shape\
            and transformation_matrix_current.shape==(4,4),\
            'wrong transformation shape'
    transition_current = transformation_matrix_current[:3,-1]
    transition_goal = transformation_matrix_goal[:3,-1]
    error = transition_goal - transition_current
    return error

def linear_interpolate(x,y,interpolate_num=10):
    '''element-wise linear interpolate function, only work for row vector, return shape=(itp_num, N)'''
    assert x.shape == y.shape, 'x, y shape mismatch'
    assert len(x.shape) < 3, 'x should be less than 2 dimension'
    if len(x.shape) == 2:
        assert x.shape[0] == 1, 'x and y are not row vector'
    if len(x.shape) == 1:
        x = x.reshape(1,-1)
        y = y.reshape(1,-1)
        

    if interpolate_num is not 0:
        step = np.arange(0., 1., 1/interpolate_num).reshape(-1,1)
    else:
        return x.reshape((1,-1))
    interpolate_vec = np.tile(x.squeeze(), (step.shape[0],1))
    interpolate_vec = interpolate_vec + step @ (y - x).squeeze().reshape(1,-1)
    

    interpolate_vec = interpolate_vec[:interpolate_num,:]
    

    interpolate_vec = np.vstack((interpolate_vec, y))
    return interpolate_vec
    
def transmat2xyzRxRyRz(transmat):
    '''transformation matrix to xyzRxRyRz, euler angles are in randians'''
    assert isinstance(transmat, np.ndarray), 'wrong transmat dtype, should be ndarray'
    assert transmat.shape == (4,4), 'wrong transmat shape, should be (4,4)'
    xyz = transmat[:3,-1]
    rot_mat = transmat[:3,:3]
    rotation = R.from_matrix(rot_mat)
    RxRyRz = rotation.as_euler('xyz', degrees=False)
    xyzRxRyRz = np.append(xyz,RxRyRz)
    return xyzRxRyRz
    
def xyzRxRyRz2transmat(xyzRxRyRz):
    '''xyzRxRyRz target transform into (4,4) transformation matrix, make ensure euler is in randians'''
    assert isinstance(xyzRxRyRz, np.ndarray), 'wrong xyzRxRyRz dtype, should be ndarray'
    assert xyzRxRyRz.shape == (6,), 'wrong xyzRxRyRz.shape, should be (6,)'
    transmat = np.identity(4, dtype=np.float32)
    xyzRxRyRz = xyzRxRyRz.reshape(6,)
    rotation = R.from_euler('xyz', xyzRxRyRz[3:], degrees=False)
    target_orientation = rotation.as_matrix()
    target_position = xyzRxRyRz[:3]
    transmat[:3,:3] = target_orientation
    transmat[:3,-1] = target_position
    return transmat  
         
def safe_track_ICOP(theta0, DH, xyzRxRyRz0, xyzRxRyRz1, EOA2tool):
    assert xyzRxRyRz0.shape == xyzRxRyRz1.shape and len(xyzRxRyRz0.shape) == 1, 'wrong xyzRxRyRz shape'
    assert len(theta0.shape) == 1 and theta0.shape[0] == DH.shape[0], 'wrong theta0 shape'
    diagonal_elements = [1.,1.,1.,1.,1.,1.]
    assert len(diagonal_elements) == DH.shape[0], 'it is not a GP50 robot, please cancel this diagonal element and use identity instead'
    H = np.diag(diagonal_elements)
    f = -H.T @ theta0.reshape(-1,1)
    end_pos, _ = forward_kinematics(theta0, DH, EOA2tool=EOA2tool)
    J = jacobian(theta0, DH, DH.shape[0], end_pos, EOA2tool=EOA2tool)
    Aeq = J[:3,:]
    beq = ((xyzRxRyRz0 - xyzRxRyRz1)[:3] + (J[:3,:] @ theta0.reshape(-1,1)).squeeze()).reshape(-1,1)
    lb = -np.ones(theta0.shape, dtype=np.float32)*np.pi
    ub = np.ones(theta0.shape, dtype=np.float32)*np.pi
    theta_new = solve_qp(P=csc_matrix(H), q=f, A=csc_matrix(Aeq), b=beq, lb=lb, ub=ub, solver="osqp")
    return theta_new

def ICOP_EOAT_tracking(theta_ref, DH, xyzRxRyRz_goal, EOA2tool=np.identity(4,dtype=np.float32), interpolate_step_size = 0.00002):
    _, EOAT_M = forward_kinematics(theta_ref, DH, EOA2tool=EOA2tool)
    xyzRxRyRz_ref = transmat2xyzRxRyRz(EOAT_M)

    def interpolate(xyzRxRyRz_ref, xyzRxRyRz_goals):
        goal0 = xyzRxRyRz_goal[0,:]
        interpolated_xyzRxRyRz = np.zeros((0, 6))
        ref2goal0 = linear_interpolate(xyzRxRyRz_ref, goal0, interpolate_step_size)
        interpolated_xyzRxRyRz = np.vstack((interpolated_xyzRxRyRz, ref2goal0))
        for i in range(xyzRxRyRz_goals.shape[0]-1):
            goal2goalnext = linear_interpolate(xyzRxRyRz_goals[i,:], xyzRxRyRz_goals[i+1,:], interpolate_step_size)
            interpolated_xyzRxRyRz = np.vstack((interpolated_xyzRxRyRz, goal2goalnext))
        return interpolated_xyzRxRyRz
    
    interpolated_xyzRxRyRz = interpolate(xyzRxRyRz_ref, xyzRxRyRz_goal)
    assert interpolated_xyzRxRyRz.shape[1] == 6
    robot_traj = np.zeros((DH.shape[0], 0))
    robot_traj = np.hstack((robot_traj, theta_ref.reshape((-1,1))))
    for _, xyzRxRyRz_goal in tqdm(enumerate(interpolated_xyzRxRyRz[1:,:]),\
                                        total=interpolated_xyzRxRyRz.shape[0]-1,\
                                        desc="ICOP tracking interpolated xyzRxRyRz goals",\
                                        ncols=100):


        theta_current = robot_traj[:,-1]
        _, transformation_matrix_current = forward_kinematics(theta_current, DH, EOA2tool=EOA2tool)
        xyzRxRyRz_current = transmat2xyzRxRyRz(transformation_matrix_current)
        transformation_matrix_goal = xyzRxRyRz2transmat(xyzRxRyRz_goal)
        error = compute_error(transformation_matrix_current, transformation_matrix_goal)
        iter_num = 0
        
        while(1):
            if iter_num > max_iter:
                print(f"ICOP failed during tracking goal")
                import ipdb; ipdb.set_trace()
                return robot_traj.T
                exit()
            
            theta_current = safe_track_ICOP(theta_current, DH, xyzRxRyRz_current, xyzRxRyRz_goal, EOA2tool=EOA2tool)
            assert theta_current.shape[0] == DH.shape[0], 'wrong solved QP theta shape'
            _, transformation_matrix_current = forward_kinematics(theta_current, DH, EOA2tool=EOA2tool)
            xyzRxRyRz_current = transmat2xyzRxRyRz(transformation_matrix_current)
            
            error = compute_error(transformation_matrix_current, transformation_matrix_goal)
            iter_num += 1
            

            if(abs(error[0]) <= eps[0] and abs(error[1]) <= eps[1] and abs(error[2]) <= eps[2]):
                break
        
    return robot_traj.T

def interpolate_joint_traj():
    '''linear interpolate joint trajectories'''
    traj = np.loadtxt("auto_cali_configuration_trajectory/current_traj.txt")
    try:
        assert traj.shape[1] == 6
    except:
        assert traj.shape[0] == 6, 'only works for 6DOF for now'
        traj = traj.T
    


    assert traj.shape[0] >= 2, f'traj shape is {traj.shape}'
    traj_itp = np.zeros((0,traj.shape[1]))
    for i in range(traj.shape[0]-1):
        s = traj[i,:]
        e = traj[i+1,:]

        s = s / 180. * np.pi 
        e = e / 180. * np.pi
        

        if i == 0:

            traj_itp_tmp = linear_interpolate(s,e,interpolate_num=int(max(abs((s-e))) / (interpolate_radius_size * 10)))
        else:
            traj_itp_tmp = linear_interpolate(s,e,interpolate_num=int(max(abs((s-e))) / interpolate_radius_size))
        traj_itp = np.vstack((traj_itp, traj_itp_tmp))
    

    np.savetxt("auto_cali_configuration_trajectory/current_traj_itp.txt", traj_itp, fmt='%.5f')

def interpolate_joint_traj_stable_scan():
    '''linear interpolate joint trajectories, input traj is in radius'''
    traj = np.loadtxt("auto_cali_configuration_trajectory/current_traj.txt")
    try:
        assert traj.shape[1] == 6
    except:
        assert traj.shape[0] == 6, 'only works for 6DOF for now'
        traj = traj.T
    
    assert traj.shape[0] >= 2, f'traj shape is {traj.shape}'
    traj_itp = np.zeros((0,traj.shape[1]))
    for i in range(traj.shape[0]-1):
        s = traj[i,:]
        e = traj[i+1,:]
        if i == 0:
            traj_itp_tmp = linear_interpolate(s,e,interpolate_num=int(max(abs((s-e))) / (interpolate_radius_size * 10)))
        else:
            traj_itp_tmp = linear_interpolate(s,e,interpolate_num=int(max(abs((s-e))) / interpolate_radius_size))
        
        stable_e = np.tile(e, (stable_num,1))
        traj_itp = np.vstack((traj_itp, traj_itp_tmp))
        traj_itp = np.vstack((traj_itp, stable_e))
    np.savetxt("auto_cali_configuration_trajectory/current_traj_itp.txt", traj_itp, fmt='%.5f')

def icop_start2goal(theta_start, 
                    theta_goal, 
                    interpolate_step_size,
                    optimized_EOA2laser = np.array([[ 0.46696576, -0.85304677, -0.23292516, -0.04035923],
                                                    [-0.75904047, -0.52181047,  0.38932168,  0.08113769],
                                                    [-0.45365238, -0.00500027, -0.89116472, -0.5751366 ],
                                                    [ 0.,          0.,          0.,          1.        ]])):
    '''ICOP track the start pose to goal pose, input is raudius, return shape = (N,DOF)'''
    GP50_DH = np.array([[0, 0.540, 0.145, -np.pi/2],
                   [np.pi/2, 0, -0.87, np.pi],
                   [0, 0, -0.21, np.pi/2],
                   [0, -1.025, 0, -np.pi/2],
                   [0, 0, 0, np.pi/2],
                   [0, -0.175, 0, 0]])

    
    _, transmat = forward_kinematics(theta_goal, GP50_DH, optimized_EOA2laser)
    movement_targets = transmat2xyzRxRyRz(transmat)
    
    robot_traj = ICOP_EOAT_tracking(theta_start, GP50_DH, movement_targets.reshape((1,-1)), optimized_EOA2laser, interpolate_step_size=interpolate_step_size)
    
    if abs(robot_traj[-1,:] - theta_goal).max() > joint_change_safe_step_size:
        itp_theta = linear_interpolate(robot_traj[-1,:],\
                                        theta_goal,\
                                        interpolate_num=int(abs(robot_traj[-1,:]\
                                                                - theta_goal).max()\
                                                                    /interpolate_radius_size))
        robot_traj = np.vstack((robot_traj, itp_theta))
    return robot_traj

def GP_tp_scan_traj():
    '''ICOP track the ref pose to goal pose'''
    GP50_DH = np.array([[0, 0.540, 0.145, -np.pi/2],
                   [np.pi/2, 0, -0.87, np.pi],
                   [0, 0, -0.21, np.pi/2],
                   [0, -1.025, 0, -np.pi/2],
                   [0, 0, 0, np.pi/2],
                   [0, -0.175, 0, 0]])
    
    




    optimized_EOA2laser = np.array([[ 0.46696576, -0.85304677, -0.23292516, -0.04035923],
                                    [-0.75904047, -0.52181047,  0.38932168,  0.08113769],
                                    [-0.45365238, -0.00500027, -0.89116472, -0.5751366 ],
                                    [ 0.,          0.,          0.,          1.        ]])
   
    
    theta_start = np.array([0.86369944, -0.07286952, -0.22518887, -1.11716783, -0.699404, -0.80573249])
    theta_goal = np.array([0.93990749,  0.05803017, -0.09511806, -1.08629704, -0.63179922, -0.85976166])
    
    _, transmat = forward_kinematics(theta_goal, GP50_DH, optimized_EOA2laser)
    movement_targets = transmat2xyzRxRyRz(transmat)
    
        
    robot_traj = ICOP_EOAT_tracking(theta_start, GP50_DH, movement_targets.reshape((1,-1)), optimized_EOA2laser, 0.002)
    print(f'the optimized robot trajectory is {robot_traj / np.pi * 180}')
    np.savetxt("auto_cali_configuration_trajectory/current_traj.txt", robot_traj, fmt='%.5f')
  
def add_stable_points():
    traj = np.loadtxt("auto_cali_configuration_trajectory/scan_traj_012201.txt")
    new_traj = traj[0]
    for i in range(traj.shape[0]):
        if i % 200 == 0:
            stable_points = np.tile(traj[i], (stable_num,1))
        else:
            stable_points = traj[i]
        new_traj = np.vstack((new_traj, stable_points))
    
    np.savetxt("auto_cali_configuration_trajectory/scan_traj_stable_012201.txt", new_traj, fmt='%.5f')
 
def test():
    theta_current = np.array([ 0.02544278, -0.16822009, -0.35138146,  0.08825332, -0.49393135, -3.14571083])
    DH = np.array([[ 0.        ,  0.54      ,  0.145     , -1.57079633],
                [ 1.57079633,  0.        , -0.87      ,  3.14159265],
                [ 0.        ,  0.        , -0.21      ,  1.57079633],
                [ 0.        , -1.025     ,  0.        , -1.57079633],
                [ 0.        ,  0.        ,  0.        ,  1.57079633],
                [ 0.        , -0.175     ,  0.        ,  0.        ]])
    xyzRxRyRz_current = np.array([ 1.60228836,  0.08732617,  0.91078579, -2.46226826, -0.14298073,
       -0.82049797])
    xyzRxRyRz_goal = np.array([ 1.60228908,  0.08733927,  0.91078573, -2.46224694, -0.14295141,
       -0.82053664])
    EOA2tool = np.array([[ 0.46696576, -0.85304677, -0.23292516, -0.04035923],
                        [-0.75904047, -0.52181047,  0.38932168,  0.08113769],
                        [-0.45365238, -0.00500027, -0.89116472, -0.5751366 ],
                        [ 0.        ,  0.        ,  0.        ,  1.        ]])
    theta_new = safe_track_ICOP(theta_current, DH, xyzRxRyRz_current, xyzRxRyRz_goal, EOA2tool=EOA2tool)
    print(theta_new)
 
if __name__ == '__main__':
    test()



    
    












    
        
    