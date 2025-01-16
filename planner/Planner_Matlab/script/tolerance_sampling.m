
clc
clear




ROBOT = 'GP50';
robot=robotproperty(ROBOT);
[pos,M]=CapPos(robot.base,robot.DH,robot.cap);

wp2robot = 1.7;
height = -0.051;
theta = -pi/5;
transfer_wp_new;

consider_line = xp3-P1P2;
anchor_point = point_anchor_rotate;
arr = arr(:,1:60);
LineSegs = planes_cross_property(planes,consider_line);
load('results/performance_boost.mat');
theta = safe_theta(:,140);
feasible = [];
for i = 1: 10000
    v = 0.1;
    sigma = sqrt(v);
    mu = theta(4);
    X = sigma .* randn + mu;
    t4 = randn;
    if t4 < -pi || t4 > pi
        continue
    end
    
    mu = theta(5);
    X = sigma .* randn + mu;
    t5 = randn;
    if t5 < -pi/2 || t5 > pi/2
        continue
    end
    
    mu = theta(6);
    X = sigma .* randn + mu;
    t6 = randn;
    if t6 < -pi || t6 > pi
        continue
    end
    
    theta_tmp = [theta(1:3);t4;t5;t6];
    col_flag = check_collision_complete(theta_tmp,robot,planes,LineSegs,anchor_point,consider_line);
    if (col_flag == 0)
        feasible = [feasible theta_tmp];
    end
end

plot3(feasible(4,:),feasible(5,:),feasible(6,:),'*')
zlabel('z axis');
ylabel('y axis');
xlabel('x axis');

