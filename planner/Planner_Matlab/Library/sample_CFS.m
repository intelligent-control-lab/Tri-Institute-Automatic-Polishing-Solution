function CFS_theta = sample_CFS(theta0, robot, PC, PC_idx, n)
    CFS_theta = [];
    cnt = 0;
    nPC = max(abs(PC_idx));
    Sstack = [];
    Lstack = [];
    for j = 1:nPC
        curPC = PC(PC_idx == j, :);
        dist = dist_arm_PC(theta0, robot.DH, robot.base, robot.cap, curPC);
        dfunc = @(x) dist_arm_PC(x, robot.DH, robot.base, robot.cap, curPC);
        ref_grad = num_grad_jac(dfunc,theta0);
        s = dist - ref_grad*theta0;
        l = -ref_grad;
        Sstack = [Sstack;s];
        Lstack = [Lstack;l];
    end
    size(Lstack)
    size(Sstack)
    while cnt < n
        cur_theta = theta0 + 0.1 * (2*rand(6,1) - 1);
        
        if (Lstack * cur_theta >= Sstack) 
            continue;
        end
        if checkFeasible(cur_theta, PC, PC_idx, robot.DH, robot.base, robot.cap) == 0
            continue;
        end
        CFS_theta = [CFS_theta, cur_theta];
        cnt = cnt + 1;
    end
    
end

