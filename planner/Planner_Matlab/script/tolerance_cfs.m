

clc
clear

ROBOT = 'GP50';
robot=robotproperty(ROBOT);
wp2robot = 1.7;
height = -0.051;
theta = -pi/5;
transfer_wp_realsetup;
consider_line = xp3-P1P2;
anchor_point = point_anchor_rotate;
load('results/cpp_polish.mat')
As = [];
bs = [];

for i = 80:size(safe_theta,2)
    fprintf('-----------
    Sstack = [];
    Lstack = [];
    theta0 = safe_theta(:,i);
    nplanes = size(planes, 1);
    for j = 1:nplanes
        plane = planes(j,:);
        lineseg = LineSegs{j}.p;
        dist = dist_arm_plane_complete(theta0, robot.DH, robot.base, robot.cap, plane, lineseg, anchor_point, consider_line);
        dfunc = @(x) dist_arm_plane_complete(x, robot.DH, robot.base, robot.cap, plane, lineseg, anchor_point, consider_line);
        ref_grad = num_grad_jac(dfunc,theta0);
        s = dist - ref_grad*theta0;
        l = -ref_grad;
        Sstack = [Sstack;s];
        Lstack = [Lstack;l];  
    end
    As = [As {Lstack}];
    bs = [bs {Sstack}];
end