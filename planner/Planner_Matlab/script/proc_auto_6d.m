log = [];
t_wait = 40;
axis_track = 'Y';
if axis_track == 'x' || axis_track == 'X'
    axis_index = 1;
end
if axis_track == 'y' || axis_track == 'Y'
    axis_index = 2;
end
if axis_track == 'z' || axis_track == 'Z'
    axis_index = 3;
end
log_list1 = [];
log_list2 = [];
while (t <= size(robot.goal,2))
    if t > pre_t_laser
        robot.Msix2target = robot.Msix2tool;
    else
        robot.Msix2target = robot.Msix2laser;
    end
    disp(t);
    curLog = zeros(1,3);
    curLog(1) = t;
    c_next = robot.goal(1:3,t);
    c_next = setVertice(c_next', M_PC_0^(-1))';
    M_contactframe = robot.goalframe{t};
    col_flag = 1;
    theta_cur = theta_pre;
    c_cur = c_pre;
    wp_pos_cur = wp_pos_pre;
    trackT = tic;
    iteration_cnt = 1;
    max_iteration = 20;
    M_target = [M_target, M_contactframe];
    axis_target = M_contactframe(:,axis_index);
    error_log = [];
    success = 0;
    if t < pre_t_axis
        while(iteration_cnt <= max_iteration)
            iteration_cnt = iteration_cnt + 1;
            [theta_new, wp_pos_new] = safetrack_auto_robot(theta_pre, wp_pos_pre, robot, c_next, PC_origin, PC_idx);
            if size(theta_new, 1) == 0
                continue;
            end
            [c_new, rot_new] = ForKine(theta_new, robot.DH, robot.base, robot.Msix2target);
            M_new = axang2rotm([rot_new / norm(rot_new), norm(rot_new)]);
            PC_new= processPC(PC_origin, wp_pos_new);
            c_target = next_point_WP(wp_pos_pre, c_next, PC_origin);
            c_pre = c_new;
            theta_pre = theta_new;
            wp_pos_pre = wp_pos_new;
        end
    end
    stop_flag = 0;
    safe_theta = [safe_theta theta_pre];
    safe_wp_pos = [safe_wp_pos wp_pos_pre];
    if t >= 20
        mask(t) = 1;
    else 
        mask(t) = 0;
    end
    t=t+1;
end