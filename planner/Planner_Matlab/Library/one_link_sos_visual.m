
while (t<= tfinal + nstep + nwait)
    disp(t);
    c_next = robot.goal(1:3,t);

    col_flag = 1;
    while(norm(c_next-c_pre)>thres || col_flag == 1)
        [theta_new, need_flag, eq_diff] = safetrack(theta_pre, robot, c_next,planes,LineSegs,anchor_point,consider_line);

        c_pre = ForKine(theta_new, robot.DH, robot.base, robot.cap);
        theta_pre = theta_new;
        col_flag = check_collision_complete(theta_pre,robot,planes,LineSegs,anchor_point,consider_line);
    end
    safe_traj = [safe_traj c_pre];
    safe_theta = [safe_theta theta_pre];

    t=t+1;
end