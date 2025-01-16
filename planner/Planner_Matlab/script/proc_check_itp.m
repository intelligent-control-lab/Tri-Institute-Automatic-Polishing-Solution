
success = 1;
c0 = ForKine(theta_pre, robot.DH, robot.base, robot.Msix2target);
c1 = next_point_WP(wp_pos_cur, c_next, PC_origin);
if norm(c1 - c0) > thres
    disp("P0")
    success = 0;
end
if check_collision_complete_PC_cluster(theta_pre, robot, PC, PC_idx) == 1
    disp("P1")
    success = 0;
end
if checkFeasible(theta_pre, PC, PC_idx, robot.DH, robot.base, robot.cap, robot.Msix2tool) == 0
    disp("P2")
    success = 0;
end


diff_wp_pos = wp_pos_pre - wp_pos_cur;
diff_theta = theta_pre - theta_cur;
steps = 10;
minDist = inf;

PC_step = processPC(PC_origin, wp_pos_cur);
for s=0:steps
    theta_step = theta_cur + s * diff_theta /steps;
    if norm(diff_wp_pos) > 0
        wp_pos_step = wp_pos_cur + s * diff_wp_pos /steps;
        PC_step = processPC(PC_origin, wp_pos_step);
    end
    [col_flag, dist_step] = check_collision_complete_PC_cluster(theta_step, robot, PC_step, PC_idx);
    if col_flag == 1
        success = 0;
        disp("P3")
        s
        break;
    end
    if checkFeasible(theta_step, PC_step, PC_idx, robot.DH, robot.base, robot.cap, robot.Msix2tool) == 0
        success = 0;
        break;
    end
end