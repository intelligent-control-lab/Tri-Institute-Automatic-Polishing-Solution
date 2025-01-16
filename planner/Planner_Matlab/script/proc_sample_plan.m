batch_num = 1;
batch_cnt = 0;
batch_size = 50;
theta_opt = [];
c_opt = [];

while batch_cnt < batch_num
    
    theta_sampled = [];
    while size(theta_sampled,2) < batch_size
        disp("****************Sampling*****************")
        theta_tmp = theta_cur + [0;0;1*(2*rand - 1);2*(2*rand - 1);2*(2*rand - 1);2*(2*rand - 1)];
        
        if checkFeasible(theta_tmp, PC, PC_idx, robot.DH, robot.base, robot.cap) == 0
            continue;
        end
        theta_sampled = [theta_sampled, theta_tmp];
    end
    
    batch_cnt = batch_cnt + 1;

    theta_final = zeros(size(theta_sampled));
    distList = -1*ones(1, batch_size);
    diffList = 10000*ones(1, batch_size);
    collision = zeros(1, batch_size);
    parfor i=1:batch_size
        disp("****************Tracking*****************")
        iteration_cnt = 1;
        col_flag = 1;
        theta_pre = theta_sampled(:, i);
        c_pre = ForKine(theta_pre, robot.DH, robot.base, robot.cap);

        while(norm(c_next-c_pre)>thres || col_flag == 1)
            if iteration_cnt > 20
                break
            end
            iteration_cnt = iteration_cnt + 1;
            [theta_new, need_flag, eq_diff] = safetrack_PC_cluster(theta_pre, robot, c_next, PC, PC_idx);

            if size(theta_new, 1) == 0
                continue;
            end
            c_pre = ForKine(theta_new, robot.DH, robot.base, robot.cap);
            theta_pre = theta_new;
            [col_flag, ~] = check_collision_complete_PC_cluster(theta_pre, robot, PC, PC_idx);
        end

        if iteration_cnt > 20
            collision(i) = 1;
        end
        if (norm(theta_pre - theta_cur) > 3)
            collision(i) = 1;
        end

        if collision(i) == 0
            diff = theta_pre - theta_cur;
            steps = 100;
            minDist = inf;
        
            for s=1:steps
                theta_step = theta_cur + s * diff /steps;
                [col_flag, dist_step] = check_collision_complete_PC_cluster(theta_step, robot, PC, PC_idx);
                if col_flag == 1
                    collision(i) = 1;
                    break;
                end
                if checkFeasible(theta_step, PC, robot.DH, robot.base, robot.cap) == 0
                    collision(i) = 1;
                    break;
                end
                minDist = min(minDist, dist_step);
            end
        
            if collision(i) == 0
                disp("****************Tracking Solved*****************")
                theta_final(:,i) = theta_pre;
                distList(i) = minDist;
                diffList(i) = norm(theta_pre - theta_cur)
            end   
        end

        
    end
    [maxD, maxIdx] = max(distList);
    [maxD, maxIdx] = min(diffList);
    if (maxD == 10000) 
        continue;
    else 
        success = 1;
        theta_pre = theta_final(:,maxIdx);
        c_pre = ForKine(theta_pre, robot.DH, robot.base, robot.cap);
        break;
    end
end



