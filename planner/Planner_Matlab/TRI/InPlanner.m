clc
clear
close all;
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
exp_mode = 'robot';
track_mode_main = 'robot';
track_solver = 'sqp';
track_mode_sample = 'robot';
explore_mode = 'None';
sample_mode = 'robot';
nullspace_mode = 'cmaes';
thres = 0.001;
pre_t_axis = 20;
pre_t_laser = 1000;
offset = 0.00;
angle_z = 0.0;
angle_y = 0.0;
theta_init = [0;-2.3; 0.8; -0.5;0.5;-pi/2];
wp_pos_init = [0;1.4;0];
[PC_origin, PC_idx] = load_PC;
[PC, M_PC, base_point, center_point] = processPC(PC_origin, wp_pos_init);
load('data/weld_new/weld_in.mat');
arr_axis3 = weld_in';
arr = setVertice(arr_axis3', M_PC)';
arr_raw = setVertice(arr', inv(M_PC))';
ref_point = mean(arr_raw, 2)';
normvec = extract_norm_vector(PC_origin, arr_raw, ref_point);
normvec = setVertice(normvec, M_PC)' - M_PC(1:3,4);
circle_norm = cross(normvec(:,2), normvec(:,1));

for i=1:size(arr,2)
    arr(:,i) = arr(:,i) + offset * normvec(:,i);
end
track_center = 1;
ReferenceTrajectoryCenter;
size(arr)
size(robot.goal)
[c_init, rot_init] = ForKine(theta_init, robot.DH, robot.base, robot.Msix2target);
M_init = axang2rotm([rot_init / norm(rot_init), norm(rot_init)]);
pre_n = size(robot.goal,2) - size(arr,2);
robot.goalframe = {};
cf_init = M_init';
contact_frame_list = [cf_init(:)'];
for i=1:size(robot.goal,2)
    if i > pre_n
        vec_x_fake = circle_norm;
        vec_z = normvec(:,i - pre_n);
        vec_y = cross(vec_z, vec_x_fake);
        vec_x = cross(vec_y, vec_z);
        d_M = eul2rotm([0, angle_y, angle_z],'XYZ');
        robot.goalframe{i} = [vec_x/norm(vec_x), vec_y/norm(vec_y),vec_z/norm(vec_z)] * d_M;
        
    else
        robot.goalframe{i} = M_init;
    end
    cf = robot.goalframe{i}';
    contact_frame_list = [contact_frame_list;cf(:)'];
end
mask = -1*ones(size(robot.goal,2),1);
t=1;
theta_pre = theta_init;
wp_pos_pre = wp_pos_init;
c_pre = ForKine(theta_init, robot.DH, robot.base, robot.Msix2tool);
M_PC_0 = M_PC;
safe_traj = [c_pre];
safe_theta = [theta_pre];
safe_wp_pos = [wp_pos_pre];
M_target = M_init;
visualize_traj_realsetup_PC(safe_theta, robot.goal, safe_wp_pos, M_target);
startT = tic;
proc_auto_6d;
T = toc(startT)
size(safe_theta)
visualize_traj_realsetup_PC(safe_theta,robot.goal, safe_wp_pos, M_target);
safe_theta_real = safe_theta;
safe_theta_real(2,:) = safe_theta_real(2,:) + pi/2;
mask = [0;mask];
save_path = 'TRI/results/in_measure.txt';
dlmwrite(save_path,safe_theta_real');
save_path_mask = 'TRI/results/in_measure_mask.txt';
dlmwrite(save_path_mask,mask);
















