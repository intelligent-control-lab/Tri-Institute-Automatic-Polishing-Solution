batch_num = 1;
batch_cnt = 0;
batch_size = 50;
theta_opt = [];
c_opt = [];

while batch_cnt < batch_num
    
    theta_sampled = [];
    wp_pos_sampled = [];
    while size(theta_sampled,2) < batch_size
        disp("****************Sampling*****************")

        theta_tmp = theta_cur;
        wp_pos_tmp = wp_pos_cur + [0;0.1*(2*rand - 1);0.3*(2*rand - 1)];

        PC_tmp = processPC(PC_origin, wp_pos_tmp);
        if checkFeasible(theta_tmp, PC_tmp, robot.DH, robot.base, robot.cap) == 0
            continue;
        end
        theta_sampled = [theta_sampled, theta_tmp];
        wp_pos_sampled = [wp_pos_sampled, wp_pos_tmp];
    end
    
    batch_cnt = batch_cnt + 1;

    theta_final = zeros(size(theta_sampled));
    wp_pos_final = zeros(size(wp_pos_sampled));

    distList = -1*ones(1, batch_size);
    diffList = 10000*ones(1, batch_size);
    collision = zeros(1, batch_size);
    parfor i=1:batch_size
        disp("****************Tracking*****************")
        iteration_cnt = 1;
        col_flag = 1;
        theta_pre = theta_sampled(:, i);
        wp_pos_pre = wp_pos_sampled(:, i);
        c_pre = ForKine(theta_pre, robot.DH, robot.base, robot.cap);

        while(1)
            if iteration_cnt > 20
                break
            end
            iteration_cnt = iteration_cnt + 1;
            [theta_new, wp_pos_new, need_flag, eq_diff] = safetrack_PC_cluster_v2(theta_pre, wp_pos_pre, robot, c_next, PC_origin, PC_idx);

            if size(theta_new, 1) == 0
                continue;
            end
            
            c_pre = ForKine(theta_new, robot.DH, robot.base, robot.cap);
            theta_pre = theta_new;
            wp_pos_pre = wp_pos_new;
    
            PC_new= processPC(PC_origin, wp_pos_pre);
            col_flag = check_collision_complete_PC_cluster(theta_pre, robot, PC_new, PC_idx);
            c_target = next_point_WP(wp_pos_pre, c_next, PC_origin);
            
            if norm(c_target-c_pre)<= thres && col_flag == 0
                break;
            end
        end

        if iteration_cnt > 20
            collision(i) = 1;
        end
        if (norm(theta_pre - theta_cur) > 3) || (norm(wp_pos_pre - wp_pos_cur) > 1) 
            collision(i) = 1;
        end

        if collision(i) == 0
            diff_theta = theta_pre - theta_cur;
            diff_wp_pos = wp_pos_pre - wp_pos_cur;
            steps = 20;
            minDist = inf;
        
            for s=1:steps
                theta_step = theta_cur + s * diff_theta /steps;
                wp_pos_step = wp_pos_cur + s * diff_wp_pos /steps;
                PC_step = processPC(PC_origin, wp_pos_step);
                [col_flag, dist_step] = check_collision_complete_PC_cluster(theta_step, robot, PC_step, PC_idx);
                if col_flag == 1
                    collision(i) = 1;
                    break;
                end
                if checkFeasible(theta_step, PC_step, robot.DH, robot.base, robot.cap) == 0
                    collision(i) = 1;
                    break;
                end
                minDist = min(minDist, dist_step);
            end
        
            if collision(i) == 0
                disp("****************Tracking Solved*****************")
                theta_final(:,i) = theta_pre;
                wp_pos_final(:,i) = wp_pos_pre;
                distList(i) = minDist;
                diffList(i) = norm(theta_pre - theta_cur)
            end   
        end

        
    end
    [maxD, maxIdx] = min(diffList);
    if (maxD == 10000) 
        continue;
    else 
        success = 1;
        theta_pre = theta_final(:,maxIdx);
        wp_pos_pre = wp_pos_final(:,maxIdx);
        c_pre = ForKine(theta_pre, robot.DH, robot.base, robot.cap);
        break;
    end
end



