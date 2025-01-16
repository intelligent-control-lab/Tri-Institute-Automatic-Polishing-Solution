
function [theta_new, wp_pos_new] = safetrack_auto_robot(theta0, wp_pos0,robot,c1,PC_origin, PC_idx)
if nargin < 7
    consider_line = 1.2;
end
PC = processPC(PC_origin, wp_pos0);
wp_pos_new = wp_pos0;
c1 = next_point_WP(wp_pos0, c1, PC_origin);
c0 = ForKine(theta0, robot.DH, robot.base, robot.Msix2target);
H = blkdiag(5,5,2,1,5,10);
if PC_idx == 0
    disp("here")
    H = blkdiag(5,5,100,100,100,100);
end
f = -H'*theta0;
Sstack = [];
Lstack = [];
Jac = Jacobi(theta0,robot.DH,robot.nlink,c0);
Diff = Jac(1:3,:);
Aeq = Diff;
beq = c1 - c0 + Diff * theta0;
theta_new = quadprog(H,f,Lstack,Sstack,Aeq,beq,robot.lb,robot.ub);
end