

function robot=robotproperty(id)
robot.name = id;
switch id    
        case 'GP50'

        robot.nlink=6;
        robot.umax=10;
        robot.delta_t=0.5;
        
        robot.lb = [-pi;-pi;-pi;-pi;-pi;-pi];
        robot.ub = [pi;pi;pi;pi;pi;pi];







        robot.DH=[0, 0.281, 0.145, -pi/2;
                  -pi/2, 0, 0.87, 0;
                  0, 0, 0.21, -pi/2;
                  0, 1.025, 0, pi/2;
                  0, 0, 0, -pi/2;
                  0, 0.175, 0, 0];
              
        robot.dfDH=[0, 0.281, 0.145, -pi/2;
                  -pi/2, 0, 0.87, 0;
                  0, 0, 0.21, -pi/2;
                  0, 1.025, 0, pi/2;
                  0, 0, 0, -pi/2;
                  0, 0.66139, 0, 0];

           
        robot.base=[0;0;0.259];
        robot.cap={};



        robot.cap{1}.p = [-0.145 -0.145;0.105 0.105;0 0];
        robot.cap{1}.r = 0.385;
        
        robot.cap{2}.p = [-0.87 0;0 0;-0.1945 -0.1945];
        robot.cap{2}.r = 0.195;
        
        robot.cap{3}.p = [-0.02 -0.09;0.073 0.073;0.115 0.115];
        robot.cap{3}.r = 0.33;
        
        robot.cap{4}.p = [0 0;-0.65 0;-0.0235 -0.0235];
        robot.cap{4}.r = 0.115;
        
        robot.cap{5}.p = [0 0;0.0145 0.0145;0.025 0.025];
        robot.cap{5}.r = 0.15;

        

























        tool_angle = 22.70;
        tool_base_angle = -180+58.88;
        tool_len_1 = 0.2021;
        tool_len_2 = 0.39476;
        laser_len = 0.45;
        tool_angle = tool_angle / 180 * pi;
        robot.cap{6}.p1 = [0,0;0,0;-0.1,-tool_len_1];
        robot.cap{6}.p2 = [0,-tool_len_2*sin(tool_angle);0,0;-tool_len_1,-tool_len_1-tool_len_2*cos(tool_angle)];
        robot.cap{6}.p3 = [0,0.13;0,0;-tool_len_1,-tool_len_1];
        robot.cap{6}.p4 = [0,0.13;0,0;-tool_len_1-0.06,-tool_len_1-0.06];





        robot.cap{6}.r1 = 0.04;
        robot.cap{6}.r2 = 0.09;
        robot.cap{6}.r3 = 0.04;
        robot.cap{6}.r4 = 0.04;
        robot.cap{6}.num = 5;
        matlab_Msix2tool = eye(4);

        M_matlab2real = eye(4);
        M_matlab2real(1:3,1:3) = eul2rotm([pi, 0, pi],'XYZ');
        
        tool_base_angle = tool_base_angle / 180 * pi;
        R_offset = [tool_base_angle, -tool_angle, 0];
        T_offset = [-tool_len_1*sin(tool_angle) * cos(tool_base_angle), -tool_len_1*sin(tool_angle)* sin(tool_base_angle),  tool_len_2 + tool_len_1*cos(tool_angle)];
        d_M = eul2rotm(R_offset,'zyx');
        matlab_Msix2tool(1:3,1:3) = matlab_Msix2tool(1:3,1:3)*d_M;
        matlab_Msix2tool(1:3,4) =T_offset;







        real_Msix2tool = [[ 0.476382, -0.848996, -0.228614, -0.036112];
                            [-0.789593, -0.52748 ,  0.313543,  0.062358];
                            [-0.386785,  0.031146, -0.921643, -0.57754 ];
                            [ 0.      ,  0.      ,  0.      ,  1.      ]];

        robot.Msix2tool = M_matlab2real * real_Msix2tool;
        Msix2laser =[[ 0.476382, -0.848996, -0.228614,  0.030721];
                    [-0.789593, -0.52748 ,  0.313543, -0.040823];
                    [-0.386785,  0.031146, -0.921643, -0.431957];
                    [ 0.      ,  0.      ,  0.      ,  1.      ]];
        
        Mlaser2target = eye(4);
        Mlaser2target(1:3,4) = [0;0;laser_len];
        robot.Msix2laser = M_matlab2real * Msix2laser * Mlaser2target;


        Msix2laser_point1 = M_matlab2real * Msix2laser;
        Mlaser2target(1:3,4) = [0;0;laser_len - 0.04];
        Msix2laser_point2 = M_matlab2real * Msix2laser * Mlaser2target;
        Mlaser2target(1:3,4) = [0.07;0;0];
        Msix2laser_point3 = M_matlab2real * Msix2laser * Mlaser2target;

        laser_point1 = [Msix2laser_point1(1:3,4);1];
        laser_point2 = [Msix2laser_point2(1:3,4);1];
        laser_point3 = [Msix2laser_point3(1:3,4);1];
        laser_point1 = inv(robot.Msix2tool)*laser_point1;
        laser_point2 = inv(robot.Msix2tool)*laser_point2;
        laser_point3 = inv(robot.Msix2tool)*laser_point3;
        robot.cap{6}.p5 = [laser_point1(1:3),laser_point2(1:3)];
        robot.cap{6}.r5 = 0.001;

        robot.cap{6}.p6 = [laser_point2(1:3),laser_point3(1:3)];
        robot.cap{6}.r6 = 0.001;

        robot.cap{6}.num = 4;


        robot.Msix2target = robot.Msix2tool;


end


robot.A=[eye(robot.nlink) robot.delta_t*eye(robot.nlink);
        zeros(robot.nlink) eye(robot.nlink)];
robot.B=[0.5*robot.delta_t^2*eye(robot.nlink);
        robot.delta_t*eye(robot.nlink)];
robot.Ac=[eye(3) robot.delta_t*eye(3);
        zeros(3) eye(3)];
robot.Bc=[0.5*robot.delta_t^2*eye(3);
        robot.delta_t*eye(3)];


end