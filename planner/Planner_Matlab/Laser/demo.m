clc
clear

ROBOT = 'GP50';
robot=robotproperty(ROBOT);
[PC_origin, PC_idx] = load_PC;
safe_theta_1 = [0;-2.3; 0.5; 0;0.5;-2.0];
safe_theta_2 = [1;-1; 0.5; 0;0.5;-2.0];
safe_theta_3 = [-1;-1; 0.5; 0;0.5;-2.0];
safe_theta_4 = [1;-1.5; 0.5; 1;0.5;-2.0];
safe_theta_5 = [-1;-1; 0.5; 0;1.5;-2.0];
safe_theta = [safe_theta_1, safe_theta_2, safe_theta_3, safe_theta_4, safe_theta_5];
n = 5
safe_theta = []
for i=1:n

    safe_theta = [safe_theta, [0;0;0;0;0;0]];
end

wp_pos_pre = [0;0;0];
c_next = [1.5;0;0.5];

Msix2tool = robot.Msix2tool;
for j=1:size(safe_theta, 2)
    c_next_cur = c_next + [0.1*(2*rand(2,1) - 1); 0];
    safe_theta_init = safe_theta(:,1) + 1*(2*rand(6,1) - 1);
    safe_theta(:,j) = IK(safe_theta_init, c_next_cur, robot);
    DH = robot.DH;
    nlink=size(DH,1);
    DH(:,1) = safe_theta(:,j);
    M=cell(1,nlink+1); M{1}=eye(4);
    for i=1:nlink
        if i <= nlink
            R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
                sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
                0  sin(DH(i,4)) cos(DH(i,4))];
            T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
            M{i+1}=M{i}*[R T; zeros(1,3) 1];
        end
        if i == nlink
            M{i+1}=M{i+1}*Msix2tool;
        end
    end
    M{i+1}(1:3,4) + robot.base;
end
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
options = optimoptions('fmincon','Display','off','FiniteDifferenceType', 'central');

[laser_pos,fval,exitflag] = fmincon(@(x)obj(x, robot, safe_theta),[0.6;-0.5;-0.1;0;0;0],A,b,Aeq,beq,lb,ub,[],options);
laser_pos
visualize(safe_theta);

function theta = IK(init_theta, c1, robot)
    theta = init_theta;
    cnt = 0;
    while cnt < 10000
        c0 = ForKine(theta, robot.DH, robot.base, robot.cap);
        dc = c1 - c0;
        if norm(dc)<0.0001
            break;
        end
        Jac = Jacobi(theta,robot.DH,robot.nlink,c0 - robot.base);
        Jac = Jac([1,2,3],:);
        dtheta = pinv(Jac)*dc;
        theta = theta + 0.01*dtheta;
        cnt = cnt + 1;
    end
end

function loss = obj(x, robot, safe_theta)
    Msix2tool = robot.Msix2tool;

    Msix2tool(1:3,4) = x(4:6);
    Msix2tool(1:3,1:3) = eul2rotm(x(1:3)');
    laser_point = [];
    for j=1:size(safe_theta,2)
        j
        DH = robot.DH;
        nlink=size(DH,1);
        DH(:,1) = safe_theta(:,j);
        M=cell(1,nlink+1); M{1}=eye(4);
    
         for i=1:nlink
            if i <= nlink
                R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
                    sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
                    0  sin(DH(i,4)) cos(DH(i,4))];
                T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
                M{i+1}=M{i}*[R T; zeros(1,3) 1];
            end
            if i == nlink
                M{i+1}=M{i+1}*Msix2tool;
            end
         end
         cur_point =  M{i+1}(1:3,4) + robot.base;
         laser_point = [laser_point,cur_point];
    end
    laser_mean = mean(laser_point,2);
    laser_vector = laser_point - laser_mean;

    loss = mean(abs(laser_vector(3,:)))
end