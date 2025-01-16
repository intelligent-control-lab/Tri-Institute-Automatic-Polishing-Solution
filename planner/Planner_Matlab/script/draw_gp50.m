



addpath('Library')
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
DH = robot.DH;
jointn = 1;
orig = DH(jointn,1);

safe_theta = robot.DH(:,1);
safe_theta(3)= safe_theta(3) - pi/4;
safe_theta(5)= safe_theta(5) + pi/2;
for step = 1:size(safe_theta, 2)
    theta = safe_theta(:,step);
    DH(:,1) = theta;
    [pos,M]=CapPos(robot.base,DH,robot.cap);
    color = [1,0.5,0.2];
    handle=[];
    points = 20;
    for i = 1:size(pos,2)
        posi = pos{i};
        r1 = posi.p(:,1);
        r2 = posi.p(:,2);
        R = posi.r;
        if r1 == r2
            [X,Y,Z] = sphere(points);
            X = X*R + r1(1);
            Y = Y*R + r1(2);
            Z = Z*R + r1(3);
        else

            N = 12;
            [X,Y,Z] = cylinder2P(R, N,r1',r2');
        end

        surf(X,Y,Z);
        hold on
    end
    view([1,-0.5,0.4]);
    xlim=[-1,2];
    ylim=[-1,2];
    zlim=[0,2];
    axis([xlim,ylim,zlim]);
    lighting=camlight('left');
    set(gca,'Color',[0.8 0.8 0.8]);
    wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[0.5,0.5,0.5]);
    wall{2}.handle=fill3([xlim(1),xlim(1),xlim(1),xlim(1)],[ylim(1),ylim(1),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0,0.9,0.9]);
    wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(2),ylim(2),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0,0.9,0.9]);
    axis equal
    
    

    hold on
    planes = [-0.8553854681017643, -0.0005463239779652795, -1, 2.9257986256071304;
              0.05748143290163125, -1, 0.06252308341911939, -0.3026991195122747;
              -0.7329844887892633, 0.017746299409831523, -1, 2.462147522592861;
              -0.10475571509951867, -1, -0.13116945145812048, 0.49306524145584574];

    for i = 1:size(planes,1)
        if planes(i,3) == -1
            x1 = 1.2:0.1:2.3;
            y1 = -0.205:0.05:0.205;
            [x, y] = meshgrid(x1,y1);
            z = planes(i,1)*x + planes(i,2)*y + planes(i,4);
        end
        if planes(i,2) == -1
            x1 = 1.2:0.1:2.3;
            z1 = 0.8:0.02:1.8;
            [x, z] = meshgrid(x1,z1);
            y = planes(i,1)*x + planes(i,3)*z + planes(i,4);
        end   
        surf(x,y,z);
        hold on 
    end
    point = [1.427542310177686513e+00, -1.554129442733900024e-01, 1.459791645304504115e+00];
    plot3(point(1), point(2), point(3), '*');


end

