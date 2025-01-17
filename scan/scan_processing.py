import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import glob
from detection_magic import single_weld_automatic_extract, interpolate_target_area_after_polish, filter_data
import os
from tqdm import tqdm
from scipy.optimize import minimize, least_squares
from scipy.spatial.transform import Rotation as R
from matplotlib import rcParams
import math
rcParams['font.family'] = 'serif'

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from detection_magic import TextColor

'''
The global parameters for the detection
'''
y_scale = 0.1
vmin = 160
vmax = 600
polish_force = 10.

pingpong_radius = 15
skip_num = 0

polish_iteration = 10
visual_coordinate = False
display_intermediate_target = True
weld_visualization = True


scan_frame_skip, remove_height_each_iteration, print_command = 1, 0, True
np.set_printoptions(precision=6, suppress=True)
EOA2laser_ref= np.array([   [1, 0, 0, 0.0852],
                            [0, 1, 0, -0.0559],
                            [0, 0, 1, -0.2523],
                            [0, 0, 0, 1]])
EOA2laser_SC3012_final=np.array([[ 0.494599, -0.849144, -0.18527 ,  0.034391],
                                [-0.805251, -0.527928,  0.269936, -0.039756],
                                [-0.327024,  0.01568 , -0.944886, -0.427299],
                                [ 0.      ,  0.      ,  0.      ,  1.      ]])
EOA2Tool_ref=np.array(
[[1.,    0.,     1.,     -0.0489],
 [0.,    1.,     0.,     0.0885],
 [0.,   0.,     1.,     -0.5699],
 [0.0,   0.0,    0.0,    1.0]]    
)
optimized_EOA2Tool=np.array([[ 0.494599, -0.849144, -0.18527 , -0.0256  ],
                            [-0.805251, -0.527928,  0.269936,  0.056391],
                            [-0.327024,  0.01568 , -0.944886, -0.580479],
                            [ 0.      ,  0.      ,  0.      ,  1.      ]])

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

def weld_height(target_pc_full, weld_pc_full):
    """Extract the maximum height of current weld

    Args:
        target_pc_full (ndarray): the point cloud ideal target weld after polishing 
        weld_pc_full (ndarray): the point cloud of detected weld before polishing 
    """
    vector_collection = target_pc_full - weld_pc_full
    assert vector_collection.shape[1] == 3
    euclidean_norm = np.linalg.norm(vector_collection, axis=1).squeeze()
    mean_weld_height = euclidean_norm.mean()
    vars_weld_height = euclidean_norm.std()

    maximum_weld_height = np.partition(euclidean_norm, -10)[-10]
    return mean_weld_height, vars_weld_height, maximum_weld_height

def GP50_FK(theta, 
            laser_frame_data, 
            EOA2T= np.array([[1, 0, 0, 0.0852],
                            [0, 0, -1, -0.0559],
                            [0, 1, 0, -0.2523],
                            [0, 0, 0, 1]])):
    """tansform the laser frame data into the world frame, EOA2Laser transformation is estimated via CAD model

    Args:
        theta (ndarray): robot joint configuration, shape = (DOF,)
        laser_frame_data (ndarray): the point of interest in the laser frame, shape = (3,N), N is the number of points in the laser frame
        EOA2T (ndarray): the EOA2T transformation matrix, shape = (4,4), default value is hand measured

    Returns:
        world_frame_data (ndarray): point of interest in the world frame, shape = (3,N), value is represented in meters
    """
    DH = np.array([[0, 0.540, 0.145, -np.pi/2],
                   [np.pi/2, 0, -0.87, np.pi],
                   [0, 0, -0.21, np.pi/2],
                   [0, -1.025, 0, -np.pi/2],
                   [0, 0, 0, np.pi/2],
                   [0, -0.175, 0, 0]])

    nlink = 6
    DH[0:6, 0] = DH[0:6, 0] + theta
    DH_rows = DH.shape[0]

    M = []
    M.append(np.identity(4))


    for i in range(DH_rows):
        R = np.array([[np.cos(DH[i, 0]), np.sin(DH[i, 0]) * np.cos(DH[i, 3]), np.cos(DH[i, 0]) * np.sin(DH[i, 3])],
                      [np.sin(DH[i, 0]) * np.cos(DH[i, 3]), np.sin(DH[i, 0]),  np.cos(DH[i, 0]) * np.sin(DH[i, 3])],
                      [0, np.sin(DH[i, 3]), np.cos(DH[i, 3])]])

        T = np.array([DH[i, 2] * np.sin(DH[i, 0]), DH[i, 3] * np.cos(DH[i, 0]), DH[i, 1]])

        M.append(np.dot(M[i], np.vstack([np.hstack([R, T.reshape(-1, 1)]), np.array([0, 0, 0, 1])])))

    assert len(laser_frame_data.shape) == 2 and laser_frame_data.shape[0] == 3, "check laser frame data format, should be two dimension, 3*N" 
    laser_frame_data = np.vstack((laser_frame_data / 1000, np.ones(laser_frame_data.shape[1]).reshape(1,-1)))
    laser_origin_2_world_frame = M[-1] @ EOA2T
    world_frame_data = (laser_origin_2_world_frame @ laser_frame_data)[:3,:]
    return world_frame_data

def visual_weld_whole(weld_pc, nonweld_pc, target_pc, mid_target_pc_full, target_of_interest):
    """
    visualize the whole scan of nonweld part and weld part using different color
    """
    def o3dpc_whole(pc, color):
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pc)
        if color == 'r':
            point_cloud.paint_uniform_color([1, 0, 0])
        if color == 'black':
            point_cloud.paint_uniform_color([0, 0, 0])
        if color == 'g':
            point_cloud.paint_uniform_color([0, 1, 0])
        if 'b+' in color:
            point_cloud.paint_uniform_color([0, 1 - eval(color.replace('b+', '')) / polish_iteration, eval(color.replace('b+', '')) / polish_iteration])
        return point_cloud
    
    def axes_pc_generation(nonweld_point_cloud):
        """
        nonweld point cloud will always exist
        """
        bbox = nonweld_point_cloud.get_axis_aligned_bounding_box()
        bbox_diag_length = bbox.get_max_bound() - bbox.get_min_bound()
        coordinate_frame_size = max(bbox_diag_length) * 5
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=coordinate_frame_size)
        return axes

    def generate_random_points_around_center(center, num_points=100, radius=0.002):

        theta = np.random.uniform(0, 2*np.pi, num_points)
        phi = np.random.uniform(0, np.pi, num_points)


        x = center[0] + radius * np.sin(phi) * np.cos(theta)
        y = center[1] + radius * np.sin(phi) * np.sin(theta)
        z = center[2] + radius * np.cos(phi)


        random_points = np.column_stack((x, y, z))

        return random_points

    if weld_pc is not None:

            nonweld_point_cloud = o3dpc_whole(nonweld_pc, 'black')
            target_point_cloud = o3dpc_whole(target_pc, 'g')
            axes = axes_pc_generation(nonweld_point_cloud)
            if not display_intermediate_target:
                '''
                display the whole scan and different intermediate weld goals, and final target weld
                '''

                weld_point_cloud = o3dpc_whole(weld_pc, 'r')
                intermediate_target_point_clouds = []
                for i in range(len(mid_target_pc_full.keys())):
                    mid_target_name = f'mid_target_{i}'
                    intermediate_target_point_clouds.append(o3dpc_whole(mid_target_pc_full[mid_target_name], 'b+'+f'{i+1}'))
                pc_full_list = [weld_point_cloud, nonweld_point_cloud, target_point_cloud, axes] if visual_coordinate else [weld_point_cloud, nonweld_point_cloud, target_point_cloud]
                pc_full_list.extend(intermediate_target_point_clouds)
                
            else:
                '''
                display the whole scan, final target weld, and the next intermediate goal with specific points 
                '''

                intermediate_target_point_cloud = o3dpc_whole(mid_target_pc_full['mid_target_0'][::2], 'b+'+f'{1}')
                
                target_of_interest_augment = np.zeros((0,3))

                for idx in range(target_of_interest.shape[0]):

                    tgoi_aug = generate_random_points_around_center(center=target_of_interest[idx, :])
                    target_of_interest_augment = np.vstack((target_of_interest_augment, tgoi_aug.reshape((-1,3))))
                
                target_of_interest_point_cloud = o3dpc_whole(target_of_interest_augment, 'r')
                pc_full_list = [nonweld_point_cloud, axes] if visual_coordinate else [nonweld_point_cloud]
                pc_full_list.append(intermediate_target_point_cloud)
                pc_full_list.append(target_of_interest_point_cloud)

                
            o3d.visualization.draw_geometries(pc_full_list) 
            
    else:

        nonweld_point_cloud = o3dpc_whole(nonweld_pc, 'b')
        axes = axes_pc_generation(nonweld_point_cloud)
        if visual_coordinate:
            o3d.visualization.draw_geometries([nonweld_point_cloud, axes]) 
        else:
            o3d.visualization.draw_geometries([nonweld_point_cloud]) 

def raw_scan_world_frame(EOA2laser=EOA2laser_ref):
    """
    visualize raw scan with transformation to the world frame
    """ 
    path = 'scan_data_temporary/'
    idx_world_frame = np.zeros((3,0))
    for idx, f in enumerate(glob.glob(path+'*.txt')):
        data = np.loadtxt(f'{path}{idx}.txt')
        indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
        y = data[1,indices].squeeze()
        z = data[2,indices].squeeze()
        x = np.zeros_like(y)
        xyz_camera_frame = np.row_stack((x,y,z))
        theta = data[0,:6]
        idx_world_frame = np.hstack((idx_world_frame, GP50_FK(theta, xyz_camera_frame, EOA2T=EOA2laser)))

    idx_world_frame = idx_world_frame.T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(idx_world_frame)
    point_cloud.paint_uniform_color([0, 0, 0])
    o3d.visualization.draw_geometries([point_cloud])

def raw_scan_recover_snap(root_path):
    path = f'{root_path}0/'
    path_laser_snap = f'{root_path}laser_snap/0/EOA2laser.txt'
    EOA2laser = np.loadtxt(path_laser_snap)
    idx_world_frame = np.zeros((3,0))

    for idx, f in enumerate(glob.glob(path+'*.txt')):
        data = np.loadtxt(f'{path}{idx}.txt')


        indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
        y = data[1,indices].squeeze()
        z = data[2,indices].squeeze()
        x = np.zeros_like(y)
        xyz_camera_frame = np.row_stack((x,y,z))

        theta = data[0,:6]

        idx_world_frame = np.hstack((idx_world_frame, GP50_FK(theta, xyz_camera_frame, EOA2T=EOA2laser)))


    idx_world_frame = idx_world_frame.T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(idx_world_frame)

    point_cloud.paint_uniform_color([0, 0, 0])

    o3d.visualization.draw_geometries([point_cloud])

def raw_before_after_frame(after_path, before_path, EOA2laser=EOA2laser_ref):
    """
    visualize raw scan with transformation to the world frame
    """ 
    path = after_path
    idx_world_frame = np.zeros((3,0))

    for idx, f in enumerate(glob.glob(path+'*.txt')):
        data = np.loadtxt(f'{path}{idx}.txt')
        indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
        y = data[1,indices].squeeze()
        z = data[2,indices].squeeze()
        x = np.zeros_like(y)
        xyz_camera_frame = np.row_stack((x,y,z))
        theta = data[0,:6]
        idx_world_frame = np.hstack((idx_world_frame, GP50_FK(theta, xyz_camera_frame, EOA2T=EOA2laser)))

    idx_world_frame = idx_world_frame.T
    point_cloud_after = o3d.geometry.PointCloud()
    point_cloud_after.points = o3d.utility.Vector3dVector(idx_world_frame)
    point_cloud_after.paint_uniform_color([0, 0, 0])
    path = before_path
    idx_world_frame = np.zeros((3,0))

    for idx, f in enumerate(glob.glob(path+'*.txt')):
        data = np.loadtxt(f'{path}{idx}.txt')
        indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
        y = data[1,indices].squeeze()
        z = data[2,indices].squeeze()
        x = np.zeros_like(y)
        xyz_camera_frame = np.row_stack((x,y,z))
        theta = data[0,:6]
        idx_world_frame = np.hstack((idx_world_frame, GP50_FK(theta, xyz_camera_frame, EOA2T=EOA2laser)))


    idx_world_frame = idx_world_frame.T
    point_cloud_before = o3d.geometry.PointCloud()
    point_cloud_before.points = o3d.utility.Vector3dVector(idx_world_frame)
    point_cloud_before.paint_uniform_color([0, 0, 1])

    o3d.visualization.draw_geometries([point_cloud_before, point_cloud_after])

def save_raw_scan_cloud_point_registration(EOA2laser=EOA2laser_ref):
    """
    save all the raw scan of cloud point over workpiece 
    """ 
    idx_world_frame = np.zeros((3,0))
    paths = ['data/workpiece_head_SC3012/pc2/',]
    for _, path in enumerate(paths):
        for idx, f in enumerate(glob.glob(path+'*.txt')):
            data = np.loadtxt(f'{path}{idx}.txt')
            indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
            y = data[1,indices].squeeze()
            z = data[2,indices].squeeze()
            x = np.zeros_like(y)
            xyz_camera_frame = np.row_stack((x,y,z))
            theta = data[0,:6]
            idx_world_frame = np.hstack((idx_world_frame, GP50_FK(theta, xyz_camera_frame, EOA2T=EOA2laser)))
    idx_world_frame = idx_world_frame.T
    np.savetxt('txt_results/wp_ccp_head_2.txt', idx_world_frame, fmt='%.5f')

def visual_polish_weld_result(after_path, before_path, save_path, EOA2laser=EOA2laser_ref):
    """visaulize the polished weld result, path has the original data, 
       save_path has the processed data

    Args:
        after_path (string): path saving the raw scan data point after polishing 
        before_path (string): path saving the raw scan data point before polishing 
        save_path (string): path saves the weld start and end index
        EOA2laser (ndarray): EOA2laser transformation matrix
    """
    idx_world_frame = np.zeros((3,0))

    for idx, f in enumerate(glob.glob(after_path+'*.txt')):
        if idx == 0:
            continue
        data = np.loadtxt(f'{after_path}{idx}.txt')
        indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
        y = data[1,indices].squeeze()
        z = data[2,indices].squeeze()
        x = np.zeros_like(y)
        xyz_camera_frame = np.row_stack((x,y,z))
        theta = data[0,:6]
        idx_world_frame = np.hstack((idx_world_frame, GP50_FK(theta, xyz_camera_frame, EOA2T=EOA2laser)))
    idx_world_frame = idx_world_frame.T
    weld_world_frame = np.zeros((3,0))
    weld_start_end_idx_frame = np.loadtxt(f'{save_path}initial_scan/weld_start_end_idx_frame.txt', dtype=int)
    valid_weld_frame = np.loadtxt(f'{save_path}initial_scan/valid_weld_frame.txt', dtype=int)
    for vld_frm_idx, idx in enumerate(valid_weld_frame):

        raw_data = np.loadtxt(before_path+f"{idx}.txt")
        

        raw_data_weld = raw_data[:,weld_start_end_idx_frame[vld_frm_idx][0]:weld_start_end_idx_frame[vld_frm_idx][1]]
    
        raw_data_weld = filter_data(raw_data_weld)
        raw_weld_y = raw_data_weld[1,:]
        raw_weld_z = raw_data_weld[2,:]
        
        raw_weld_x = np.zeros_like(raw_weld_y)
        xyz_camera_frame = np.row_stack((raw_weld_x,raw_weld_y,raw_weld_z))

        theta = raw_data[0,:6]

        weld_world_frame = np.hstack((weld_world_frame, GP50_FK(theta, xyz_camera_frame, EOA2T=EOA2laser)))
    weld_world_frame = weld_world_frame.T
    point_cloud_after = o3d.geometry.PointCloud()
    point_cloud_after.points = o3d.utility.Vector3dVector(idx_world_frame)

    point_cloud_after.paint_uniform_color([0, 0, 0])
    point_cloud_before = o3d.geometry.PointCloud()
    point_cloud_before.points = o3d.utility.Vector3dVector(weld_world_frame)
    point_cloud_before.paint_uniform_color([0.3, 0.2, 0.9])

    o3d.visualization.draw_geometries([point_cloud_after, point_cloud_before])

def visualize_weld_and_good_threshold(weld_height, good_thres):
    x = np.arange(len(weld_height))
    plt.plot(x, weld_height, marker='o', label='weld_height Values')
    plt.axhline(y=good_thres, color='red', linestyle='--', label='Horizontal Line at y=1')
    plt.xlabel('Index')
    plt.ylabel('weld_height Values')
    plt.title('1D weld_height Visualization with Horizontal Line')
    plt.legend()
    plt.show()

def weld_extraction_magic_world_frame(path, save_path, EOA2laser=EOA2laser_SC3012_final, good_weld_thres=0.0004):  
    """
    Intelligently findout the weld and nonweld scan over the all scan frames.
    Visualization will directly take place in the world frame 
    """
    weld_process_data_path = f'{save_path}initial_scan/'
    
    def find_first_common_element(array1, array2):
        '''find the first common element in two array'''
        for element in array1:
            if element in array2:
                return element
        return None
    
    for idx in tqdm(range(1, len(glob.glob(os.path.join(path, '*.txt'))), scan_frame_skip), desc="Weld Processing", ncols=100):

        try:
            raw_data = np.loadtxt(path+f"{idx}.txt")
        except:
            continue
        ttl_len = len(glob.glob(os.path.join(path, '*.txt')))
        skip_frame = []
        skip_frame = [1]

        if idx in skip_frame:
            continue
        theta = raw_data[0,:6]
        ttl_pts = raw_data.shape[1]
        data = raw_data[:, int(0.4* ttl_pts) : int(0.7 * ttl_pts)]
        try:
            weld_y, weld_z, nonweld_y, nonweld_z = single_weld_automatic_extract(data)
        except:
            continue
        
        indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
        raw_y = data[1,indices].squeeze()
        raw_z = data[2,indices].squeeze()
        raw_x = np.zeros_like(raw_y)

        if 'raw_scan_pc_full' not in locals():
            raw_scan_pc_full = np.zeros((0, 3))
        
        if 'target_of_interest' not in locals():
            target_of_interest = np.zeros((0,3))

            
        if 'valid_weld_frame' not in locals():
            valid_weld_frame = []
                
        if 'weld_start_end_idx_frame' not in locals():
            weld_start_end_idx_frame = []
            
        raw_scan_frame_laser = np.column_stack((raw_x, raw_y, raw_z))


        raw_scan_frame_world = GP50_FK(theta, raw_scan_frame_laser.T, EOA2T=EOA2laser)
        raw_scan_pc_full = np.vstack((raw_scan_pc_full,raw_scan_frame_world.T))
        
        if weld_y is not None:
            target_z = interpolate_target_area_after_polish(weld_y, nonweld_y, nonweld_z)
            assert target_z.shape[0] == weld_z.shape[0]
            weld_x = np.zeros_like(weld_y)
            nonweld_x = np.zeros_like(nonweld_y)
        
            if 'weld_pc_full' not in locals():
                weld_pc_full = np.zeros((0, 3))
            weld_points_frame_laser = np.column_stack((weld_x, weld_y, weld_z))
            weld_points_frame_world = GP50_FK(theta, weld_points_frame_laser.T, EOA2T=EOA2laser)
            weld_pc_full = np.vstack((weld_pc_full,weld_points_frame_world.T))
            weld_target = weld_points_frame_world[:,weld_points_frame_world.shape[1] // 2]
            target_of_interest = np.vstack((target_of_interest, weld_target.reshape((1,-1))))
            if 'nonweld_pc_full' not in locals():
                nonweld_pc_full = np.zeros((0, 3))
            nonweld_points_frame_laser = np.column_stack((nonweld_x, nonweld_y, nonweld_z))

            nonweld_points_frame_world = GP50_FK(theta, nonweld_points_frame_laser.T, EOA2T=EOA2laser)
            nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame_world.T))
            

            if 'target_pc_full' not in locals():
                target_pc_full = np.zeros((0, 3))
            target_points_frame_laser = np.column_stack((weld_x, weld_y, target_z))

            target_points_frame_world = GP50_FK(theta, target_points_frame_laser.T, EOA2T=EOA2laser)
            target_pc_full = np.vstack((target_pc_full, target_points_frame_world.T))
            

            if 'weld_middle_point_idx' not in locals():
                weld_middle_point_idx = []
            weld_middle_point_idx.append(weld_pc_full.shape[0] - weld_points_frame_world.shape[1] // 2)
            valid_weld_frame.append(idx)
            weld_start_indx = find_first_common_element(np.where(raw_data[1,:] == weld_y[0])[0], \
                                                            np.where(raw_data[2,:] == weld_z[0])[0]).squeeze()
            weld_end_indx = find_first_common_element(np.where(raw_data[1,:] == weld_y[-1])[0], \
                                                            np.where(raw_data[2,:] == weld_z[-1])[0]).squeeze()
            weld_start_end_idx_frame.append(np.array([weld_start_indx, weld_end_indx]))

        else:

            nonweld_x = np.zeros_like(nonweld_y)
            

            if 'nonweld_pc_full' not in locals():
                nonweld_pc_full = np.zeros((0, 3))
            nonweld_points_frame_laser = np.column_stack((nonweld_x, nonweld_y, nonweld_z))

            nonweld_points_frame_world = GP50_FK(theta, nonweld_points_frame_laser.T, EOA2T=EOA2laser)
            nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame_world.T))
            
    if 'weld_pc_full' in locals():
        mid_target_pc_full = {}
        mid_target_pc_full['mid_target_0'] = weld_pc_full 
        if 'weld_pc_full' in locals():

            if not os.path.exists(weld_process_data_path):
                os.makedirs(weld_process_data_path)

            np.savetxt(f"{weld_process_data_path}valid_weld_frame.txt", np.asarray(valid_weld_frame).reshape(-1,1), fmt='%d')
            np.savetxt(f"{weld_process_data_path}weld_start_end_idx_frame.txt", np.asarray(weld_start_end_idx_frame), fmt='%d')
            np.savetxt(f"{weld_process_data_path}polish_target.txt", target_of_interest, fmt='%.5f')
        else:
            raise ValueError("no valid weld detected!!!")


    if weld_visualization:
        visual_weld_whole(weld_pc_full if 'weld_pc_full' in locals() else None, \
                            nonweld_pc_full, \
                            target_pc_full if 'target_pc_full' in locals() else None, \
                            mid_target_pc_full if 'mid_target_pc_full' in locals() else None, \
                            target_of_interest if 'target_of_interest' in locals() else None)  

def weld_extraction_magic_world_frame_showcase(path, EOA2laser=EOA2laser_SC3012_final):  
    """
    Intelligently findout the weld and nonweld scan over the all scan frames.
    Visualization will directly take place in the world frame 
    """

    def find_first_common_element(array1, array2):
        '''find the first common element in two array'''
        for element in array1:
            if element in array2:
                return element
        return None
    
    for idx in tqdm(range(1, len(glob.glob(os.path.join(path, '*.txt'))), scan_frame_skip), desc="Weld Processing", ncols=100):

        raw_data = np.loadtxt(path+f"{idx}.txt")
        

        theta = raw_data[0,:6]
        


        ttl_pts = raw_data.shape[1]


        data = raw_data[:, int(0.01* ttl_pts) : int(0.99 * ttl_pts)]
        


        

        try:
            weld_y, weld_z, nonweld_y, nonweld_z = single_weld_automatic_extract(data)
        except:
            continue
        

        indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
        raw_y = data[1,indices].squeeze()
        raw_z = data[2,indices].squeeze()
        raw_x = np.zeros_like(raw_y)

        if 'raw_scan_pc_full' not in locals():
            raw_scan_pc_full = np.zeros((0, 3))
        
        if 'target_of_interest' not in locals():
            target_of_interest = np.zeros((0,3))

            
        if 'valid_weld_frame' not in locals():
            valid_weld_frame = []
                
        if 'weld_start_end_idx_frame' not in locals():
            weld_start_end_idx_frame = []
            
        raw_scan_frame_laser = np.column_stack((raw_x, raw_y, raw_z))


        raw_scan_frame_world = GP50_FK(theta, raw_scan_frame_laser.T, EOA2T=EOA2laser)
        raw_scan_pc_full = np.vstack((raw_scan_pc_full,raw_scan_frame_world.T))
        
        if weld_y is not None:


            target_z = interpolate_target_area_after_polish(weld_y, nonweld_y, nonweld_z)
            assert target_z.shape[0] == weld_z.shape[0]
            
            weld_x = np.zeros_like(weld_y)
            nonweld_x = np.zeros_like(nonweld_y)
        

            if 'weld_pc_full' not in locals():
                weld_pc_full = np.zeros((0, 3))
            weld_points_frame_laser = np.column_stack((weld_x, weld_y, weld_z))

            weld_points_frame_world = GP50_FK(theta, weld_points_frame_laser.T, EOA2T=EOA2laser)
            weld_pc_full = np.vstack((weld_pc_full,weld_points_frame_world.T))
            

            weld_target = weld_points_frame_world[:,weld_points_frame_world.shape[1] // 2]
            target_of_interest = np.vstack((target_of_interest, weld_target.reshape((1,-1))))
            

            if 'nonweld_pc_full' not in locals():
                nonweld_pc_full = np.zeros((0, 3))
            nonweld_points_frame_laser = np.column_stack((nonweld_x, nonweld_y, nonweld_z))

            nonweld_points_frame_world = GP50_FK(theta, nonweld_points_frame_laser.T, EOA2T=EOA2laser)
            nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame_world.T))
            

            if 'target_pc_full' not in locals():
                target_pc_full = np.zeros((0, 3))
            target_points_frame_laser = np.column_stack((weld_x, weld_y, target_z))

            target_points_frame_world = GP50_FK(theta, target_points_frame_laser.T, EOA2T=EOA2laser)
            target_pc_full = np.vstack((target_pc_full, target_points_frame_world.T))
            

            if 'weld_middle_point_idx' not in locals():
                weld_middle_point_idx = []
            weld_middle_point_idx.append(weld_pc_full.shape[0] - weld_points_frame_world.shape[1] // 2)

             

            

            valid_weld_frame.append(idx)
            weld_start_indx = find_first_common_element(np.where(raw_data[1,:] == weld_y[0])[0], \
                                                            np.where(raw_data[2,:] == weld_z[0])[0]).squeeze()
            weld_end_indx = find_first_common_element(np.where(raw_data[1,:] == weld_y[-1])[0], \
                                                            np.where(raw_data[2,:] == weld_z[-1])[0]).squeeze()
            weld_start_end_idx_frame.append(np.array([weld_start_indx, weld_end_indx]))

        else:

            nonweld_x = np.zeros_like(nonweld_y)
            

            if 'nonweld_pc_full' not in locals():
                nonweld_pc_full = np.zeros((0, 3))
            nonweld_points_frame_laser = np.column_stack((nonweld_x, nonweld_y, nonweld_z))

            nonweld_points_frame_world = GP50_FK(theta, nonweld_points_frame_laser.T, EOA2T=EOA2laser)
            nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame_world.T))
            
    

    if 'weld_pc_full' in locals():

        mid_target_pc_full = {}
        mid_target_pc_full['mid_target_0'] = weld_pc_full 


    if weld_visualization:
        visual_weld_whole(weld_pc_full if 'weld_pc_full' in locals() else None, \
                            nonweld_pc_full, \
                            target_pc_full if 'target_pc_full' in locals() else None, \
                            mid_target_pc_full if 'mid_target_pc_full' in locals() else None, \
                            target_of_interest if 'target_of_interest' in locals() else None)  

def polishing_analysis():
    folder_path = '10_9_23/'
    file_names = os.listdir(folder_path)
    file_names.sort()
    for sub_folder in file_names:
        path = folder_path + sub_folder + '/'
        for idx in range(0, len(glob.glob(os.path.join(path, '*.txt'))), scan_frame_skip):
            data = np.loadtxt(path+f"{idx}.txt")
            weld_y, weld_z, nonweld_y, nonweld_z = single_weld_automatic_extract(data)
            theta = data[0,:6]
            if weld_y is not None:
                target_z = interpolate_target_area_after_polish(weld_y, nonweld_y, nonweld_z)
                assert target_z.shape[0] == weld_z.shape[0]
                weld_x = np.zeros_like(weld_y)
                nonweld_x = np.zeros_like(nonweld_y)
                if 'weld_pc_full' not in locals():
                    weld_pc_full = np.zeros((0, 3))
                weld_points_frame_laser = np.column_stack((weld_x, weld_y, weld_z))
                weld_points_frame_world = GP50_FK(theta, weld_points_frame_laser.T)
                weld_pc_full = np.vstack((weld_pc_full,weld_points_frame_world.T))
                if 'nonweld_pc_full' not in locals():
                    nonweld_pc_full = np.zeros((0, 3))
                nonweld_points_frame_laser = np.column_stack((nonweld_x, nonweld_y, nonweld_z))
                nonweld_points_frame_world = GP50_FK(theta, nonweld_points_frame_laser.T)
                nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame_world.T))
                if 'target_pc_full' not in locals():
                    target_pc_full = np.zeros((0, 3))
                target_points_frame_laser = np.column_stack((weld_x, weld_y, target_z))
                target_points_frame_world = GP50_FK(theta, target_points_frame_laser.T)
                target_pc_full = np.vstack((target_pc_full, target_points_frame_world.T))
                if 'weld_middle_point_idx' not in locals():
                    weld_middle_point_idx = []
                weld_middle_point_idx.append(weld_pc_full.shape[0] - weld_points_frame_world.shape[1] // 2)
            else:
                nonweld_x = np.zeros_like(nonweld_y)
                if 'nonweld_pc_full' not in locals():
                    nonweld_pc_full = np.zeros((0, 3))
                nonweld_points_frame_laser = np.column_stack((nonweld_x, nonweld_y, nonweld_z))
                nonweld_points_frame_world = GP50_FK(theta, nonweld_points_frame_laser.T)
                nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame_world.T))
                

        if 'weld_pc_full' in locals():
            mean_weld_height, vars_weld_height, max_wld_ht = weld_height(target_pc_full, weld_pc_full)
            print(f"the weld height mean is {mean_weld_height * 1000:.3f}mm,/n the std of weld height is {vars_weld_height * 1000: .3f},/n the maximum height is {max_wld_ht * 1000:.3f}")
            

        if 'weld_pc_full' in locals():
            del weld_pc_full
        if 'nonweld_pc_full' in locals():
            del nonweld_pc_full
        if 'target_pc_full' in locals():
            del target_pc_full
        if 'weld_middle_point_idx' in locals():
            del weld_middle_point_idx

def vis_tool_height(joints, EOA2Tool):
    """visualization of different joint angle tool tip height 

    Args:
        joints (ndarray): the joints so that end tool tip touch the same height, shape=(N,6)
        EOA2Tool (ndarray): end of arm to tool transformation matrix, shape=(4,4) 
    """
    assert joints.shape[1] == 6, f'the shape of joints are not correct, expect (N,6), got {joints.shape}'
    wld_z = []
    for i in range(joints.shape[0]):
        jnt = joints[i,:]
        offset = np.array([0.,0.,0.])
        wld_xyz = GP50_FK(jnt, offset.reshape((3,-1)), EOA2Tool)
        assert wld_xyz.shape==(3,1), f'wrong wld xyz shape, expect (3,1), get {wld_xyz.shape}'
        z = wld_xyz.flatten()[-1]
        wld_z.append(z)
    
    wld_z = np.asarray(wld_z)
    x_values = np.array([i for i in range(wld_z.shape[0])])
    plt.plot(x_values, wld_z, marker='o', linestyle='-')
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.title('1D Plot with 1D Array')
    plt.show()
    
def stick_tool_cali():
    '''Laser frame data is measured in mm'''
    data = np.loadtxt('data/tool_cali/stick/0.txt')
    
    indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
    y = data[1,indices].squeeze()
    z = data[2,indices].squeeze()
    valid_idx = np.where((z < np.min(z) + 2))
    y = y[valid_idx]
    z = z[valid_idx]
    tgt_idx = np.argmin(abs(y))
    tool_y_laser_frame = y[tgt_idx] - 7.
    tool_z_laser_frame = z[tgt_idx]
    tool_x_laser_frame = -57.
    Laser2Tool = np.hstack((np.identity(3), np.array([tool_x_laser_frame / 1000., \
                                                        tool_y_laser_frame / 1000., \
                                                        tool_z_laser_frame / 1000.]).reshape(-1,1)))
    Laser2Tool = np.vstack((Laser2Tool, np.array([0.,0.,0.,1.]).reshape(1,-1)))
    EOA2Tool = EOA2laser_SC3012_final @ Laser2Tool 
    
    print(f'the laser calibrated EOA2Tool is \n {repr(EOA2Tool)}')
    
def compute_euler():
    xyzRxRyRz = transmat2xyzRxRyRz(EOA2laser_SC3012_final)
    degree = xyzRxRyRz[3:] / np.pi * 180.
    print(degree)

def laser_tool_reverse_half_pi():
    '''special design for MATLAB planning with laser mounting perpendicular to tool'''
    dxyzRxRyRz = np.array([0.,0.,0.,0.,0.,-np.pi/2])
    dtransmat = xyzRxRyRz2transmat(dxyzRxRyRz)
    Mtool = optimized_EOA2Tool @ dtransmat
    print(f'new tool is \n {Mtool}')
    xyzRxRyRz = transmat2xyzRxRyRz(Mtool)
    degree = xyzRxRyRz[3:] / np.pi * 180.
    print(f'new euler degree is \n {degree}')
    
def test():
    pass
    
if __name__ == "__main__":
    compute_euler()


