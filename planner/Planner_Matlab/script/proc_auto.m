log = [];
while (t<= tfinal + nstep + nwait)
    disp(t);
    curLog = zeros(1,3);
    curLog(1) = t;

    c_next = robot.goal(1:3,t);
    c_next = setVertice(c_next', M_PC_0^(-1))';


    col_flag = 1;
    
    theta_cur = theta_pre;
    c_cur = c_pre;
    wp_pos_cur = wp_pos_pre;

    trackT = tic;
    iteration_cnt = 1;
    while(iteration_cnt <= 20)
        iteration_cnt = iteration_cnt + 1;

        if strcmp(track_mode_main,'robot')
            if strcmp(track_solver,'ICOP')
                disp("****************Robot ICOP Tracking*****************");
                [theta_new, wp_pos_new] = safetrack_auto_robot(theta_pre, wp_pos_pre, robot, c_next, PC_origin, PC_idx);
            else
                disp("****************Robot Fmincon Tracking*****************");
                [theta_new, wp_pos_new] = safetrack_auto_fmincon(theta_pre, wp_pos_pre, robot, c_next, PC_origin, PC_idx, track_solver);
            end
        end
        
        if strcmp(track_mode_main,'collaboration')
            disp("****************Collaboration Tracking*****************");
            [theta_new, wp_pos_new] = safetrack_auto_collaboration(theta_pre, wp_pos_pre, robot, c_next, PC_origin, PC_idx);
        end



        if size(theta_new, 1) == 0
            continue;
        end

        c_new = ForKine(theta_new, robot.DH, robot.base, robot.Msix2tool);
        PC_new= processPC(PC_origin, wp_pos_new);
        c_target = next_point_WP(wp_pos_pre, c_next, PC_origin);

        c_pre = c_new;
        theta_pre = theta_new;
        wp_pos_pre = wp_pos_new;

        col_flag = check_collision_complete_PC_cluster(theta_new, robot, PC_new, PC_idx); 
        if norm(c_target-c_pre)<= thres && col_flag == 0
            break;
        end
    end
    
    T1 = toc(trackT);
    curLog(2) = iteration_cnt;
    need_sample = 0;
    stop_flag = 0;

    T2 = 0;
    if iteration_cnt > 20 || checkFeasible(theta_pre, PC_new, PC_idx, robot.DH, robot.base, robot.cap, robot.Msix2tool) == 0 

        if strcmp(explore_mode,'None')
             stop_flag = 1;
        else
            need_sample = 1;
            success = 0;

            sampleT = tic;
            if strcmp(explore_mode,'CSsample')
                 proc_auto_sample;
            end
            if strcmp(explore_mode,'NSsample')
                 proc_auto_sample_nullspace;
            end
            if strcmp(explore_mode,'NSopt')
                 proc_nullspace_opt;
            end
            
            if success == 0
                stop_flag = 1;
            else
                PC_new= processPC(PC_origin, wp_pos_pre);
                if checkFeasible(theta_pre, PC_new, PC_idx, robot.DH, robot.base, robot.cap) == 0
                    stop_flag = 1;
                end
            end

            T2 = toc(sampleT);
        end
    end
    
    
    curLog(3) = T1;
    curLog(4) = T2;
    curLog(5) = need_sample;
    log = [log, curLog'];

    if stop_flag == 1
        break;
    end
    safe_traj = [safe_traj c_pre];
    safe_theta = [safe_theta theta_pre];
    safe_wp_pos = [safe_wp_pos wp_pos_pre];

    t=t+1;
end