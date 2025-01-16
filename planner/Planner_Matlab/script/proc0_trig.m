
while (t<= tfinal + nstep + nwait)
    disp(t);
    c_next = robot.goal(1:3,t);

    col_flag = 1;
    iteration_cnt = 1;
    while(norm(c_next-c_pre)>thres || col_flag == 1)
        if iteration_cnt > 20
            break
        end
        iteration_cnt = iteration_cnt + 1;
        theta_new = safetrack_trig(theta_pre, robot, c_next, triangles);

        c_pre = ForKine(theta_new, robot.DH, robot.base, robot.cap);
        theta_pre = theta_new;
        col_flag = check_collision_complete_trig(theta_pre,robot,triangles);
    end
    if iteration_cnt <= 20
        safe_traj = [safe_traj c_pre];
        safe_theta = [safe_theta theta_pre];
        t=t+1;
    else
        break
    end

end