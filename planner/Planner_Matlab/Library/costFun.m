function [cost] = costFun(theta0, c1, theta_pre, DH, base, cap, PC, PC_idx)


    theta0;
    c0 = ForKine(theta0, DH, base, cap);
    error_target =  norm(c0 - c1);
    error_dist = -dist_arm_PC(theta0, DH, base, cap, PC);
    error_feasible = 0;
    if checkFeasible(theta0, PC, PC_idx, DH, base, cap) == 0
        error_feasible = 1;
    end
    diff = theta0 - theta_pre;
    Q = blkdiag(5,5,2,1,5,10);  
    error_diff = diff'*Q*diff;
    cost = 100*error_target + 10000*error_feasible + 100*error_dist + 10*error_diff;
end

