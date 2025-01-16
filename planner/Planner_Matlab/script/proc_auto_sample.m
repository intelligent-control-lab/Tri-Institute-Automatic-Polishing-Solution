batch_num = 1;
batch_cnt = 0;
batch_size = 30;

while batch_cnt < batch_num
    theta_sampled = [];
    wp_pos_sampled = [];

    while size(theta_sampled,2) < batch_size
        disp("****************Sampling*****************")
        theta_tmp = theta_cur;
        wp_pos_tmp = wp_pos_cur;

        if strcmp(sample_mode,'robot') || strcmp(sample_mode,'workpiece_robot')
            theta_tmp = theta_cur + [0;0;1*(2*rand - 1);2*(2*rand - 1);2*(2*rand - 1);2*(2*rand - 1)];
        end
        if strcmp(sample_mode,'workpiece') || strcmp(sample_mode,'workpiece_robot')
            wp_pos_tmp = wp_pos_cur + [0;0.1*(2*rand - 1);0.5*(2*rand - 1)];
        end 

        PC_tmp = processPC(PC_origin, wp_pos_tmp);
        if checkFeasible(theta_tmp, PC_tmp, PC_idx, robot.DH, robot.base, robot.cap) == 0
            continue;
        end
        theta_sampled = [theta_sampled, theta_tmp];
        wp_pos_sampled = [wp_pos_sampled, wp_pos_tmp];
    end
    
    batch_cnt = batch_cnt + 1;

    theta_final = zeros(size(theta_sampled));
    wp_pos_final = zeros(size(wp_pos_sampled));

    diffList = 10000*ones(1, batch_size);
    collision = zeros(1, batch_size);

    parfor i=1:batch_size
        disp("****************Tracking*****************")
        
        col_flag = 1;
        theta_pre = theta_sampled(:, i);
        wp_pos_pre = wp_pos_sampled(:, i);
        c_pre = ForKine(theta_pre, robot.DH, robot.base, robot.cap);

        iteration_cnt = 1;
        while(iteration_cnt <= 20)
            iteration_cnt = iteration_cnt + 1;
            
            if strcmp(track_mode_sample,'robot')
                disp("****************Sampled Robot Tracking*****************");
                [theta_new, wp_pos_new] = safetrack_auto_robot(theta_pre, wp_pos_pre, robot, c_next, PC_origin, PC_idx);
            end
            if strcmp(track_mode_sample,'collaboration')
                disp("****************Sampled Collaboration Tracking*****************");
                [theta_new, wp_pos_new] = safetrack_auto_collaboration(theta_pre, wp_pos_pre, robot, c_next, PC_origin, PC_idx);
            end
            
            

            if size(theta_new, 1) == 0
                continue;
            end
            
            c_new = ForKine(theta_new, robot.DH, robot.base, robot.cap);
            PC_new= processPC(PC_origin, wp_pos_pre);
            c_target = next_point_WP(wp_pos_pre, c_next, PC_origin);
    
            c_pre = c_new;
            theta_pre = theta_new;
            wp_pos_pre = wp_pos_new;
    
            col_flag = check_collision_complete_PC_cluster(theta_pre, robot, PC_new, PC_idx);     
            
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
        
            PC_step = processPC(PC_origin, wp_pos_cur);
            for s=1:steps
                theta_step = theta_cur + s * diff_theta /steps;

                if norm(diff_wp_pos) > 0
                    wp_pos_step = wp_pos_cur + s * diff_wp_pos /steps;
                    PC_step = processPC(PC_origin, wp_pos_step);
                end
                
                [col_flag, dist_step] = check_collision_complete_PC_cluster(theta_step, robot, PC_step, PC_idx);
                if col_flag == 1
                    collision(i) = 1;
                    break;
                end
                if checkFeasible(theta_step, PC_step, PC_idx, robot.DH, robot.base, robot.cap) == 0
                    collision(i) = 1;
                    break;
                end
                minDist = min(minDist, dist_step);
            end
        
            if collision(i) == 0
                disp("****************Tracking Solved*****************")
                theta_final(:,i) = theta_pre;
                wp_pos_final(:,i) = wp_pos_pre;
                diffList(i) = norm(theta_pre - theta_cur)
            end   
        end

        
    end
    [minDiff, minIdx] = min(diffList);
    if (minDiff == 10000) 
        continue;
    else 
        success = 1;
        theta_pre = theta_final(:,minIdx);
        wp_pos_pre = wp_pos_final(:,minIdx);
        c_pre = ForKine(theta_pre, robot.DH, robot.base, robot.cap);
        break;
    end
end



