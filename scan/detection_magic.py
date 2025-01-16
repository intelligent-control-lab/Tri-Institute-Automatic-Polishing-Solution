from scipy.optimize import curve_fit
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from tqdm import tqdm

class TextColor:
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    RESET = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

SC3012 = True
vmin = 160
vmax = 600
y_scale = 5
half_window_size = 10
smooth_window_size = 5
local_curvature_maxima_window = 5
min_curvature_threshold = 0.3
max_curvature_threshold = -0.03
max_cur_lower_thrd = 0.2
polynomial_fitting = '2-degree'

curvature_extract_skip = 2
pyramid_peak_curvature_threshold = 0.5
vaid_frame_minimum_x_index_length = 50
continuous_x_gap_threshold = 0.1
ignore_start_end_percent = 0.2
abnormal_thres = 3


def visual_smooth(x, y, y_smooth):
    fig, ax = plt.subplots()
    ax.plot(x, y, label='original', color='blue')
    ax.plot(x, y_smooth, label='smoothed', color='red')
    ax.legend()
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('smooth effect')
    plt.show()
    
def moving_average_with_padding(sequence, window_size = smooth_window_size):
    """
    moving average with one dimensional convolution
    padding is needed to expand the edge, so that the start and initial smoothed points have the same scale with their origins
    """
    half_window = window_size // 2
    padded_sequence = np.pad(sequence, (half_window, half_window), mode='edge')
    smoothed_sequence = np.convolve(padded_sequence, np.ones(window_size) / window_size, mode='valid')

    smoothed_sequence = smoothed_sequence[:sequence.shape[0]]
    return smoothed_sequence

def moving_average(sequence):
    return np.convolve(sequence, np.ones(smooth_window_size)/smooth_window_size, mode='same')

def polynomial_curve(x, a, b, c):

    return a * x**2 + b * x + c

def polynomial_curve_four_degree(x, a, b, c, d, e):

    return a * x**4 + b * x**3 + c * x**2 + d * x + e

def polynomial_curve_four_degree_second_derivative(x, a, b, c, d, e):
    return 4 * 3 * a * x**2 + 3 * 2 * b * x + 2 * c

def find_local_maxima_positive(curs):
    """
    find the local maximum given the curvature sequence,
    local maximum should have a nontrivial curvature magnitude
    and it is the highest among neighbourhood  
    """

    cur_magnitude = max(curs) 
    
    window = local_curvature_maxima_window
    maxima_indexes_raw_detection = []
    maxima_raw_detection = []
    maxima_indexes = []

    if cur_magnitude > min_curvature_threshold:
        for i in range(len(curs)):


            if (curs[i] > max(curs[max(i - window, 0) : i]) if len(curs[max(i - window, 0) : i]) > 0 else True)  \
                and (curs[i] > max(curs[i + 1 : min(len(curs), i + window)]) if len(curs[i + 1 : min(len(curs), i + window)]) > 0 else True) \
                and curs[i] > min_curvature_threshold: 

                maxima_indexes_raw_detection.append(i)
                maxima_raw_detection.append(curs[i])
        
        if len(maxima_indexes_raw_detection) == 1:

            maxima_indexes = None
            return maxima_indexes, 'normal'
         
        try:
            assert len(maxima_indexes_raw_detection) >= 2
        except:
            print(f"{TextColor.GREEN}{TextColor.BOLD}The maxima index must exists given there's a turning point of the weld, check min curvature threshold.{TextColor.RESET}")
            return maxima_indexes_raw_detection, 'error'


        maxima_raw_detection_copy = maxima_raw_detection.copy()
        largest = max(maxima_raw_detection_copy)
        maxima_raw_detection_copy.remove(largest)
        second_largest = max(maxima_raw_detection_copy)

        maxima_indexes.append(curs.index(largest))
        maxima_indexes.append(curs.index(second_largest))

        maxima_indexes.sort()
    else:

        maxima_indexes = None
    return maxima_indexes, 'normal'

def find_local_maxima_old(curs):
    """
    find the local maximum given the curvature sequence,
    local maximum should have a nontrivial curvature magnitude
    and it is the highest among neighbourhood  
    only consider the negative curvature as the valid weld start and end position 
    """

    cur_magnitude = min(curs) 
    
    window = local_curvature_maxima_window
    maxima_indexes_raw_detection = []
    maxima_raw_detection = []
    maxima_indexes = []

    if cur_magnitude < max_curvature_threshold:
        for i in range(len(curs)):


            if (curs[i] < min(curs[max(i - window, 0) : i]) if len(curs[max(i - window, 0) : i]) > 0 else True)  \
                and (curs[i] < min(curs[i + 1 : min(len(curs), i + window)]) if len(curs[i + 1 : min(len(curs), i + window)]) > 0 else True) \
                and curs[i] < max_curvature_threshold: 

                maxima_indexes_raw_detection.append(i)
                maxima_raw_detection.append(curs[i])
        
        if len(maxima_indexes_raw_detection) == 1:

            maxima_indexes = None
            return maxima_indexes, 'normal'
         
        try:
            assert len(maxima_indexes_raw_detection) >= 2
        except:
            print(f"{TextColor.GREEN}{TextColor.BOLD}The maxima index must exists given there's a turning point of the weld, check min curvature threshold.{TextColor.RESET}")
            return maxima_indexes_raw_detection, 'error'


        maxima_raw_detection_copy = maxima_raw_detection.copy()
        smallest = min(maxima_raw_detection_copy)
        maxima_raw_detection_copy.remove(smallest)
        second_smallest = min(maxima_raw_detection_copy)

        maxima_indexes.append(curs.index(smallest))
        maxima_indexes.append(curs.index(second_smallest))

        maxima_indexes.sort()
    else:

        maxima_indexes = None
    return maxima_indexes, 'normal'



def find_local_maxima(curs):
    """
    find the local maximum given the curvature sequence,
    local maximum should have a nontrivial curvature magnitude
    and it is the highest among neighbourhood  
    only consider the negative curvature as the valid weld start and end position 
    """    
    window = local_curvature_maxima_window
    minima_indexes_raw_detection = []
    minima_raw_detection = []
    

    max_cur = max(curs)
    if max_cur < max_cur_lower_thrd:

        return None, 'normal'
    

    max_idx = np.argmax(curs)
    

    for i in range(len(curs)):


        if (curs[i] < min(curs[max(i - window, 0) : i]) if len(curs[max(i - window, 0) : i]) > 0 else True)  \
            and (curs[i] < min(curs[i + 1 : min(len(curs), i + window)]) if len(curs[i + 1 : min(len(curs), i + window)]) > 0 else True):

            minima_indexes_raw_detection.append(i)
            minima_raw_detection.append(curs[i])
            

    minima_indexes_raw_detection = np.asarray(minima_indexes_raw_detection)
    try:
        left_minima_idx = max(minima_indexes_raw_detection[minima_indexes_raw_detection < max_idx])
    except: 
        return None, 'normal'
    
    try:
        right_minima_idx = min(minima_indexes_raw_detection[minima_indexes_raw_detection > max_idx])
    except: 
        return None, 'normal'

    
    minima_indexes = [left_minima_idx, right_minima_idx]
    return minima_indexes, 'normal'


def visual_raw(x,z):
    """
    raw visualization of the point cloud as a whole
    """
    point_cloud = o3d.geometry.PointCloud()
    points = np.column_stack((x, np.zeros_like(x) ,z))
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([point_cloud])
    
def visual_weld(weld_x,weld_y, nonweld_x, nonweld_y):
    """
    visualize the nonweld part and weld part using different color
    """
    def o3dpc(x, z, color):
        point_cloud = o3d.geometry.PointCloud()
        points = np.column_stack((x, np.zeros_like(x) ,z))
        point_cloud.points = o3d.utility.Vector3dVector(points)
        if color == 'r':
            point_cloud.paint_uniform_color([1, 0, 0])
        if color == 'b':
            point_cloud.paint_uniform_color([0, 0, 0])
        return point_cloud

    if weld_x is not None:
        weld_point_cloud = o3dpc(weld_x, weld_y, 'r')
        nonweld_point_cloud = o3dpc(nonweld_x, nonweld_y, 'b')
        o3d.visualization.draw_geometries([weld_point_cloud, nonweld_point_cloud])
    else: 

        nonweld_point_cloud = o3dpc(nonweld_x, nonweld_y, 'b')
        o3d.visualization.draw_geometries([nonweld_point_cloud])   
    
def visual_weld_whole(weld_pc, nonweld_pc, pyramid_peak=None):
    """
    visualize the whole scan of nonweld part and weld part using different color
    """
    def o3dpc_whole(pc, color):
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pc)
        if color == 'r':
            point_cloud.paint_uniform_color([1, 0, 0])
        if color == 'b':
            point_cloud.paint_uniform_color([0, 0, 0])
        if color == 'g':
            point_cloud.paint_uniform_color([0, 1, 0])
        return point_cloud

    if weld_pc is not None:

        pc_full = []
        pc_full.append(o3dpc_whole(weld_pc, 'r'))
        pc_full.append(o3dpc_whole(nonweld_pc, 'b'))
        if pyramid_peak is not None:
            pc_full.append(o3dpc_whole(pyramid_peak, 'g'))
        o3d.visualization.draw_geometries(pc_full)
    else:

        pc_full = []
        pc_full.append(o3dpc_whole(nonweld_pc, 'b'))
        if pyramid_peak is not None:
            pc_full.append(o3dpc_whole(pyramid_peak, 'g'))
        nonweld_point_cloud = o3dpc_whole(nonweld_pc, 'b')

        o3d.visualization.draw_geometries(pc_full) 
    
def plot_cur_scan(xindx, yindx, curs, ysmoothidx = None, raw_curs_maxima_indx=None):
    assert(len(xindx) == len(yindx)) and len(yindx) == len(curs), "the length of xindx and yindx and curs are not equal"
    origin_curs = curs.copy()
    if ysmoothidx is None:
        yindx = np.array(yindx)
        yindx = yindx - min(yindx)
        scale_factor = np.linalg.norm(yindx) / np.linalg.norm(np.array(curs))
        curs = np.array(curs) * scale_factor
        yindx = yindx
        fig, ax = plt.subplots()

        ax.plot(xindx, curs, label='curvature compute', color='blue')
        ax.plot(xindx, yindx, label='weld height', color='red')
        
        if  raw_curs_maxima_indx is not None:
            raw_curs_maxima_xindx = [xindx[idx] for idx in raw_curs_maxima_indx]
            for x_value in raw_curs_maxima_xindx:
                plt.axvline(x=x_value, color='r', linestyle='--')

        ax.legend()
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_title('correspondence of weld and its curvature')
        plt.show()
    else:
        yindx = np.array(yindx)
        offset = min(yindx)
        yindx = yindx - offset
        ysmoothidx = np.array(ysmoothidx)
        ysmoothidx = ysmoothidx - offset
        scale_factor = np.linalg.norm(yindx) / np.linalg.norm(np.array(curs))
        curs = np.array(curs) * scale_factor
        yindx = yindx
        ysmoothidx = ysmoothidx
        fig, axs = plt.subplots(2, 1)
        axs[0].plot(xindx, curs, label='curvature compute', color='blue')
        axs[0].plot(xindx, yindx, label='weld height', color='red')
        axs[0].plot(xindx, ysmoothidx, label='smoothed weld height', color='green')
        if  raw_curs_maxima_indx is not None:
            raw_curs_maxima_xindx = [xindx[idx] for idx in raw_curs_maxima_indx]
            for x_value in raw_curs_maxima_xindx:
                axs[0].axvline(x=x_value, color='r', linestyle='--')
        

        axs[0].legend()
        axs[0].set_xlabel('X-axis')
        axs[0].set_ylabel('Y-axis')
        axs[0].set_title('correspondence of weld and its curvature')
        axs[1].plot(xindx, origin_curs, label='original curs', color='blue')

        if  raw_curs_maxima_indx is not None:
            for x_value in raw_curs_maxima_xindx:
                axs[1].axvline(x=x_value, color='r', linestyle='--')
          

        axs[1].axhline(y=min_curvature_threshold, color='green', linestyle='-.')
        axs[1].text(0, min_curvature_threshold, f'maxima threshld = {min_curvature_threshold}', color='green', va='bottom', ha='left')


        plt.show()

def extract_curvature(data):

    indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
    x = data[1,indices].squeeze()
    y = data[2,indices].squeeze()
    window_size = 2 * half_window_size
    indx = []
    xindx = []
    yindx = []
    curs = []
    for i in range(0, len(x) - window_size + 1):      

        x_tmp = x[i:i+window_size]   
        y_tmp = y[i:i+window_size]
        params, _ = curve_fit(polynomial_curve, x_tmp, y_tmp)
        a, b, _ = params
        curvature = 2 * a
        indx.append((i+1 + i + window_size)/2)
        curs.append(curvature)
        xindx.append(x[int((i+1 + i + window_size)/2)])
        yindx.append(y[int((i+1 + i + window_size)/2)])
        
        assert((len(curs) == len(xindx)) and (len(curs) == len(yindx)))
        
    return xindx, yindx, curs

def extract_curvature_accurate(data):

    indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
    x = data[1,indices].squeeze()
    y = data[2,indices].squeeze()
    xindx = []
    yindx = []
    curs = []
    for i in range(half_window_size, len(x) - half_window_size + 1):      

        x_tmp = x[i-half_window_size:i+half_window_size]   
        y_tmp = y[i-half_window_size:i+half_window_size]
        params, _ = curve_fit(polynomial_curve, x_tmp, y_tmp)
        a, b, _ = params
        curvature = 2 * a
        curs.append(curvature)
        xindx.append(x[i])
        yindx.append(y[i])
        assert((len(curs) == len(xindx)) and (len(curs) == len(yindx)))
        
    return xindx, yindx, curs

def extract_curvature_accurate_smooth(data):
    """Automatic compute the curvature at each data point

    Args:
        data (numpy array): the raw data of cloud point in the form numpy array

    Returns:
        xindx: the x axis of each data point
        yindx: the y axis of each data point
        ysmoothindx: the y axis of each smoothed data point 
        curs: the curvature value of each data point
    """

    indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
    x = data[1,indices].squeeze()
    y = data[2,indices].squeeze()
    y_smooth = moving_average_with_padding(y)
    assert(y.shape[0] == y_smooth.shape[0])
    xindx = []
    yindx = []
    ysmoothindx = []
    curs = []
    for i in range(half_window_size, len(x) - half_window_size + 1, curvature_extract_skip):      

        x_tmp = x[i-half_window_size:i+half_window_size]   
        y_tmp = y_smooth[i-half_window_size:i+half_window_size]
        

        points = np.hstack((x_tmp.reshape(-1,1), y_tmp.reshape(-1,1)))
        points_mean = points.mean(axis=0)

        _, _, vv = np.linalg.svd(points - points_mean)
        

        tgt_vec = vv[0] / np.linalg.norm(vv[0])
        if tgt_vec[0] < 0.:

            tgt_vec = -tgt_vec 
            

        theta = np.arctan2(tgt_vec[1], tgt_vec[0])
        

        points_new_coords = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]) @ points.T
        
        x_tmp = points_new_coords[0,:]
        y_tmp = points_new_coords[1,:]
        
        if polynomial_fitting == '2-degree':

            params, _ = curve_fit(polynomial_curve, x_tmp, y_tmp)
            

            a, b, _ = params
            curvature = 2 * a
        
        if polynomial_fitting == '4-degree':

            params, _ = curve_fit(polynomial_curve_four_degree, x_tmp, y_tmp)
            

            a, b, c, d, e = params
            curvature = polynomial_curve_four_degree_second_derivative(x[i], a, b, c, d, e)
        


        curs.append(curvature)
        xindx.append(x[i])
        yindx.append(y[i])
        ysmoothindx.append(y_smooth[i])
        
        assert((len(curs) == len(xindx)) and (len(curs) == len(yindx)))
        
    return xindx, yindx, ysmoothindx, curs

def interpolate_target_area_after_polish(weld_x, nonweld_x, nonweld_y):
    """Intelligent generation of target polished weld area after polishing 

    Args:
        weld_x (1d numpy array):  the x coordinate of the weld area cloud point over detection frame 
        nonweld_x (1d numpy array): the x coordinate of the nonweld area cloud point over detection frame
        nonweld_y (1d numpy array): the y coordinate of the nonweld area cloud point over detection frame
    """

    params, _ = curve_fit(polynomial_curve, nonweld_x, nonweld_y)
    

    a, b, c = params
    def polynomial_eval(x):
        return a * x**2 + b * x + c
    return polynomial_eval(weld_x)
    
def single_weld_automatic_extract(data):
    if SC3012:
        ttl_pts = data.shape[1]
        data = data[:, int(0.0 * ttl_pts) : int(1.0 * ttl_pts)]

    indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
    x = data[1,indices].squeeze()
    y = data[2,indices].squeeze()
    


    xindx, yindx, ysmoothindx, curs = extract_curvature_accurate_smooth(data)
    

    cur_maxima_indx, error_code = find_local_maxima(curs)
    if error_code == 'error':

        print(f"{TextColor.RED}{TextColor.BOLD}Finding local maxmum errors, now start detection.{TextColor.RESET}") 

        raw_cur_maxima_indx = cur_maxima_indx
        plot_cur_scan(xindx, yindx, curs, ysmoothindx, raw_cur_maxima_indx)  
        print(f"the max magnitude is {max(curs)}. The min magnitude is {min(curs)}.")

        exit()
        
    if cur_maxima_indx is not None:

        complete_indices = np.where(np.isin(x, np.array(xindx)[cur_maxima_indx]))[0]
        

        assert(complete_indices.shape[0] == 2), "complete_indices should always have two elements"
            
        weld_x = x[complete_indices[0]:complete_indices[1]]
        weld_y = y[complete_indices[0]:complete_indices[1]]

        nonweld_x = np.append(x[:complete_indices[0]], x[complete_indices[1]:])
        nonweld_y = np.append(y[:complete_indices[0]], y[complete_indices[1]:])
    else:

        weld_x, weld_y = None, None
        nonweld_x, nonweld_y = x, y
    
    return weld_x, weld_y, nonweld_x, nonweld_y

def plot_cur_scan_single_weld():
    path = 'data/WP_IN3/0/10.txt'

    data = np.loadtxt(path)
    ttl_pts = data.shape[1]
    data = data[:, int(0.4* ttl_pts) : int(0.8 * ttl_pts)]
    

    data = filter_data(data)

    xindx, yindx, ysmoothindx, curs = extract_curvature_accurate_smooth(data)  
    

    cur_maxima_indx, error_code = find_local_maxima(curs)
    plot_cur_scan(xindx, yindx, curs, ysmoothindx if 'ysmoothindx' in locals() else None, raw_cur_maxima_indx if 'raw_cur_maxima_indx' in locals() else None)  
    print(f"the max magnitude is {max(curs)}. The min magnitude is {min(curs)}.")

def weld_finding_single_file():

    path = 'data/WP_IN3/0/10.txt'

    data = np.loadtxt(path)
    

    ttl_pts = data.shape[1]
    data = data[:, int(0.4* ttl_pts) : int(0.8 * ttl_pts)]
    

    data = filter_data(data)
    

    weld_x, weld_y, nonweld_x, nonweld_y = single_weld_automatic_extract(data)
    

    visual_weld(weld_x,weld_y, nonweld_x, nonweld_y)
        
def curvature_finding_complete_scan():   
    """
    intelligently findout the weld and nonweld scan over the all scan frames, the y is faked
    """

    path = 'data/wp_pc/wp_pc_nojoint/'
        
    for idx in range(len(glob.glob(os.path.join(path, '*.txt')))):

        data = np.loadtxt(path+f"{idx}.txt")
        print(f"===== process file {idx}.txt =====")

        try:
            weld_x, weld_z, nonweld_x, nonweld_z = single_weld_automatic_extract(data)
        except:
            print(f'{TextColor.BOLD}{TextColor.BLUE}weld detection failed at {idx}. {TextColor.RESET}') 
            continue

        if weld_x is not None:

            weld_y = y_scale*idx*np.ones_like(weld_x)
            nonweld_y = y_scale*idx*np.ones_like(nonweld_x)
        

            if 'weld_pc_full' not in locals():
                weld_pc_full = np.zeros((0, 3))
            weld_points_frame = np.column_stack((weld_x, weld_y , weld_z))
            weld_pc_full = np.vstack((weld_pc_full,weld_points_frame))
            

            if 'nonweld_pc_full' not in locals():
                nonweld_pc_full = np.zeros((0, 3))
            nonweld_points_frame = np.column_stack((nonweld_x, nonweld_y, nonweld_z))
            nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame))
        else:

            nonweld_y = y_scale*idx*np.ones_like(nonweld_x)
            

            if 'nonweld_pc_full' not in locals():
                nonweld_pc_full = np.zeros((0, 3))
            nonweld_points_frame = np.column_stack((nonweld_x, nonweld_y, nonweld_z))
            nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame))
        
    if 'weld_pc_full' in locals():

        visual_weld_whole(weld_pc_full, nonweld_pc_full)
    else:

        visual_weld_whole(None, nonweld_pc_full)   

def visualize_raw_x_z():

    path = 'scan_data_temporary/250.txt'
    data = np.loadtxt(path)
    indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
    x = data[1,indices].squeeze()
    y = data[2,indices].squeeze()
    
    fig, ax = plt.subplots()


    ax.plot(x, y, label='raw scan', color='blue')
    ax.legend()
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    plt.show()
        
def raw_scan_fake_y(path):   
    """
    intelligently findout the weld and nonweld scan over the all scan frames, the y is faked
    """
    for idx in range(len(glob.glob(os.path.join(path, '*.txt')))):

        data = np.loadtxt(path+f"{idx}.txt")
        print(f"===== process file {idx}.txt =====")

        try:
            indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax))
        except:
            continue
        weld_x, weld_z = None, None
        nonweld_x = data[1,indices].squeeze()
        nonweld_z = data[2,indices].squeeze()
        

        if weld_x is not None:

            weld_y = y_scale*idx*np.ones_like(weld_x)
            nonweld_y = y_scale*idx*np.ones_like(nonweld_x)
        

            if 'weld_pc_full' not in locals():
                weld_pc_full = np.zeros((0, 3))
            weld_points_frame = np.column_stack((weld_x, weld_y , weld_z))
            weld_pc_full = np.vstack((weld_pc_full,weld_points_frame))
            

            if 'nonweld_pc_full' not in locals():
                nonweld_pc_full = np.zeros((0, 3))
            nonweld_points_frame = np.column_stack((nonweld_x, nonweld_y, nonweld_z))
            nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame))
        else:

            nonweld_y = y_scale*idx*np.ones_like(nonweld_x)
            

            if 'nonweld_pc_full' not in locals():
                nonweld_pc_full = np.zeros((0, 3))
            nonweld_points_frame = np.column_stack((nonweld_x, nonweld_y, nonweld_z))
            nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame))
        
    if 'weld_pc_full' in locals():

        visual_weld_whole(weld_pc_full, nonweld_pc_full)
    else:

        visual_weld_whole(None, nonweld_pc_full)   
        
def filter_data(data):
    '''filter out thoese abnormal data points, whose z axis is not continuous wrt. its neighbour'''
    assert data.shape[0] == 3, 'wrong data shape'
    abnormal_idx = []
    for i in range(1, data.shape[1] - 1):
        if max(abs(data[2,i] - data[2,i-1]), abs(data[2,i] - data[2,i+1])) > abnormal_thres:
            abnormal_idx.append(i)
            

    data_filter = np.delete(data, abnormal_idx, axis=1)
    return data_filter
            
        
        
def detect_pyramid_peak_fake_y(path, visualize_flag=False):   
    """
    intelligently findout the peak of pyramid, the y is faked
    """ 

    for idx in tqdm(range(len(glob.glob(os.path.join(path, '*.txt')))), desc=f"Processing {path}", ncols=100):

        try:
            data = np.loadtxt(path+f"{idx}.txt")
        except:
            continue


        try:
            indices = np.where((data[2,:] > vmin) & (data[2,:] < vmax)) 
        except:

            continue       

        max_height = 0
        
        joint_pos = data[0,:6]
        x = data[1,indices].squeeze()
        z = data[2,indices].squeeze()
        try:
            assert len(x) > vaid_frame_minimum_x_index_length
        except:

            continue
        
        start = np.array([x[0], z[0]])
        end = np.array([x[-1], z[-1]])
        start2end = end - start
        
        for current_rest_indx in range(1,len(x)-1):

            current_vec = np.array([x[current_rest_indx], z[current_rest_indx]]) - start
            dot_product = np.dot(current_vec, start2end)


            norm_A = np.linalg.norm(current_vec)
            norm_B = np.linalg.norm(start2end)


            cos_theta = dot_product / (norm_A * norm_B)
            theta = np.arccos(cos_theta)
            

            dist = abs(norm_A * np.sin(theta))
            if dist > max_height:
                max_height = dist
                max_height_indx = current_rest_indx
        

        try:
            weld_x = x[max_height_indx]
            weld_z = z[max_height_indx]
            nonweld_x = np.append(x[:max_height_indx], x[max_height_indx+1:])
            nonweld_z = np.append(z[:max_height_indx], z[max_height_indx+1:])
        except:
            import ipdb; ipdb.set_trace()
        

        def compute_curvature(x, y, poi_index):
            


            assert (np.abs(x[poi_index-half_window_size+1:poi_index+half_window_size]\
                    - x[poi_index-half_window_size:poi_index+half_window_size-1])).max() < continuous_x_gap_threshold
            
            y_smooth = moving_average_with_padding(y)

            x_tmp = x[poi_index-half_window_size:poi_index+half_window_size]   
            y_tmp = y_smooth[poi_index-half_window_size:poi_index+half_window_size]
            
            if polynomial_fitting == '2-degree':

                params, _ = curve_fit(polynomial_curve, x_tmp, y_tmp)
                

                a, b, _ = params
                curvature = 2 * a
            
            if polynomial_fitting == '4-degree':

                params, _ = curve_fit(polynomial_curve_four_degree, x_tmp, y_tmp)
                

                a, b, c, d, e = params
                curvature = polynomial_curve_four_degree_second_derivative(x[poi_index], a, b, c, d, e)
            
            return curvature
        
        


        try:
            peak_curvature = compute_curvature(x, z, max_height_indx)
        except:
            continue
        if 'peak_curvature_full' not in locals():
            peak_curvature_full = np.zeros(0)
        peak_curvature_full = np.append(peak_curvature_full, peak_curvature)
        
        

        if 'joint_pos_full' not in locals():
            joint_pos_full = np.zeros((0,6))
        joint_pos_full = np.vstack((joint_pos_full, joint_pos[np.newaxis,:]))
        
        

        weld_y = y_scale*idx*np.ones_like(weld_x)
        nonweld_y = y_scale*idx*np.ones_like(nonweld_x)
    

        if 'weld_pc_full' not in locals():
            weld_pc_full = np.zeros((0, 3))
        weld_points_frame = np.column_stack((weld_x, weld_y , weld_z))
        weld_pc_full = np.vstack((weld_pc_full,weld_points_frame))
        

        if 'nonweld_pc_full' not in locals():
            nonweld_pc_full = np.zeros((0, 3))
        nonweld_points_frame = np.column_stack((nonweld_x, nonweld_y, nonweld_z))
        nonweld_pc_full = np.vstack((nonweld_pc_full, nonweld_points_frame))
        

    try:
        pyramid_peak_final = weld_pc_full[np.argmax(peak_curvature_full),:][np.newaxis,:]
    except:
        print(f"{TextColor.RED}{TextColor.BOLD} not enough valid data, now please rescan {TextColor.RESET}")
        raise ValueError(f"{TextColor.RED}{TextColor.BOLD} not enough valid data, now please rescan {TextColor.RESET}")
    print(f"{TextColor.GREEN}{TextColor.BOLD} the curvature is {np.max(peak_curvature_full)}{TextColor.RESET}")
    if np.max(peak_curvature_full) < pyramid_peak_curvature_threshold:
        print(f"{TextColor.RED}{TextColor.BOLD} no valid pyramid peak is detecte, now please rescan {TextColor.RESET}")
        raise ValueError(f"{TextColor.RED}{TextColor.BOLD} no valid pyramid peak is detecte, now please rescan {TextColor.RESET}")
        

    peak_poi_x = pyramid_peak_final[0,0]
    peak_poi_z = pyramid_peak_final[0,2]
    peak_poi_theta = joint_pos_full[np.argmax(peak_curvature_full),:]
        


    def generate_collections_around_peak(ref_point):
        x, y, z = ref_point[0], ref_point[1], ref_point[2]

        num_points = 100


        max_distance = 0.2


        random_points = np.zeros((0,3))
        for _ in range(num_points):

            dx, dy, dz = np.random.uniform(-max_distance, max_distance, 3)
            

            distance = np.sqrt((dx)**2 + (dy)**2 + (dz)**2)
            

            if distance <= max_distance:
                random_points = np.vstack((random_points, np.array([x + dx, y + dy, z + dz])[np.newaxis, :]))

    
        return random_points
    

    pyramid_peak_final_collections = generate_collections_around_peak(pyramid_peak_final.squeeze())

    if visualize_flag is True:
        if 'weld_pc_full' in locals():

            visual_weld_whole(weld_pc_full, nonweld_pc_full, pyramid_peak_final_collections)
        else:

            visual_weld_whole(None, nonweld_pc_full, pyramid_peak_final)   
    else:
        return peak_poi_x, peak_poi_z, peak_poi_theta
   
    
    
if __name__ == "__main__":
    weld_finding_single_file()
    plot_cur_scan_single_weld()
    







