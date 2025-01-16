
function [theta_new, need_flag, eq_diff] = safetrack(theta0,robot,c1,planes,LineSegs,anchor_point,consider_line)
if nargin < 7
    consider_line = 1.2;
end
c0 = ForKine(theta0, robot.DH, robot.base, robot.cap);
H = blkdiag(5,5,2,1,5,10);
f = -H'*theta0;
Sstack = [];
Lstack = [];
need_flag = 1;
need_flag = check_ineq_need(theta0, robot.DH, robot.base, robot.cap);
if need_flag == 1
    nplanes = size(planes, 1);
    for j = 1:nplanes
        plane = planes(j,:);
        lineseg = LineSegs{j}.p;
        dist = dist_arm_plane_complete(theta0, robot.DH, robot.base, robot.cap, plane, lineseg, anchor_point, consider_line);

        disp(['the closest distance is' num2str(dist)]);
        if dist > 0
            continue
        else
            dfunc = @(x) dist_arm_plane_complete(x, robot.DH, robot.base, robot.cap, plane, lineseg, anchor_point, consider_line);

            ref_grad = num_grad_jac(dfunc,theta0);
            s = dist - ref_grad*theta0;
            l = -ref_grad;
            Sstack = [Sstack;s];
            Lstack = [Lstack;l];
        end
    end
end
Jac = Jacobi(theta0,robot.DH,robot.nlink,c0);
Diff = Jac(1:3,:);
Aeq = Diff;
beq = c1 - c0 + Diff * theta0;
theta_new = quadprog(H,f,Lstack,Sstack,Aeq,beq,robot.lb,robot.ub);
eq_diff = abs(norm(Aeq*theta_new - beq));
end