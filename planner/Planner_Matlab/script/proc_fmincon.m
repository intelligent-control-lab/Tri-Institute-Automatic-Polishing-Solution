theta_tmp = theta_cur;
wp_pos_tmp = wp_pos_cur;
PC_0= processPC(PC_origin, wp_pos_cur);
theta0 = theta_tmp;
c1 = next_point_WP(wp_pos_cur, c_next, PC_origin);

dfunc = @(x) norm(x - theta_tmp);

A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

options = optimoptions('fmincon','Algorithm','sqp','Display','off',"MaxIterations",10,'FiniteDifferenceType', 'central');
[x,fval,exitflag] = fmincon(dfunc,theta0,A,b,Aeq,beq,lb,ub,@(x) mycon(x,robot,PC_0, c1),options);
theta_tmp = x;
theta_pre = theta_tmp;
success = 1;
c0 = ForKine(theta_tmp, robot.DH, robot.base, robot.cap);
if norm(c1 - c0) > thres
    disp("P0")
    success = 0;
end
if check_collision_complete_PC_cluster(theta_tmp, robot, PC_0, PC_idx) == 1
    disp("P1")
    success = 0;
end
if checkFeasible(theta_tmp, PC_0, PC_idx, robot.DH, robot.base, robot.cap) == 0
    disp("P2")
    success = 0;
end


diff_wp_pos = wp_pos_pre - wp_pos_cur;
diff_theta = theta_tmp - theta_cur;
steps = 1;
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
        break;
    end
    if checkFeasible(theta_step, PC_step, PC_idx, robot.DH, robot.base, robot.cap) == 0
        success = 0;
        break;
    end
end


function [c,ceq] = mycon(x,robot,PC, c1)
    c = -dist_arm_PC(x, robot.DH, robot.base, robot.cap, PC);
    ceq = norm(ForKine(x, robot.DH, robot.base, robot.cap) - c1);
end