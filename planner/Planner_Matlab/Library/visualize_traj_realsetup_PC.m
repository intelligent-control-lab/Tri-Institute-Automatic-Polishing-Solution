function visualize_traj_realsetup(safe_theta, arr0, wp_pos, M_target)
figure;
robot=robotproperty('GP50');
robotCAD = load(strcat('figure/',robot.name,'.mat'));
robotCAD = load(strcat('figure/gp50_new.mat'));
scale = 1;
framen = 1;
[PC_origin, PC_idx] = load_PC(1);
[~, M_PC_0] = processPC(PC_origin, wp_pos(:,1));
arr0 = setVertice(arr0', M_PC_0^(-1))';

for cnt = 1:size(safe_theta,2)

    robot.DH(:,1) = safe_theta(:,cnt);
    
    M=CapPos_visual(robot);
    clf;
    hold on;

    handle=[];

    if nargin >= 3
        [PC, M1, base_point, center_point] = processPC(PC_origin, wp_pos(:, cnt));
    else
        [PC, M1, base_point, center_point] = processPC(PC_origin);
    end
    
    

    for i=1:numel(robotCAD.workpiece)
        f=robotCAD.workpiece{i}.f; v=robotCAD.workpiece{i}.v.*scale; c=robotCAD.workpiece{i}.c; 
        color=[237,229,116]/255;
        v = setVertice(v,M1);
        handle.workpiece(i) = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None', 'FaceAlpha',.3);
    end
    hold on;
    scatter3(base_point(1), base_point(2), base_point(3), 10, 'cyan', 'filled');
    hold on;
    scatter3(center_point(1), center_point(2), center_point(3), 10, 'r', 'filled');
    hold on;

    
    arr = setVertice(arr0', M1)';



    for i=1:numel(robotCAD.base)
        f=robotCAD.base{i}.f; v=robotCAD.base{i}.v.*scale; c=robotCAD.base{i}.c; color=robotCAD.base{i}.color;
        for j=1:size(v,1) 
            v(j,:) = v(j,:);
        end
        v = setVertice(v,M{1});
        handle.base(i) = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
    end

    for i=1:5
        v=robotCAD.link{i}.v.*scale; f=robotCAD.link{i}.f; c=robotCAD.link{i}.c; 

        color=[249,212,35]/255;
        v = setVertice(v,M{i+1});


        handle.link(i) = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
    end

    
    [pos,M_0]=CapPos(robot.base,robot.DH,robot.cap, robot.Msix2tool);
    color = [rand rand rand];
    for i=1:6
        point1 = pos{i}.p(:,1);
        point2 = pos{i}.p(:,2);
        p = [point1, point2];
        plot3(p(1,:), p(2,:), p(3,:), '*-', 'Color',color);
        hold on;
    end


















































    handle = [];
    valpha = 0.5;
    color = [255,255,243]/255;
    [sx, sy, sz] = sphere(10);
    n = size(M,2);
    for i=2:n
        if i < n
            rot = M{i}(1:3,1:3);
            trans = M{i}(1:3,4);
            p = robot.cap{i-1}.p;
            c1 = (rot*p(:, 1)+trans)';
            c2 = (rot*p(:, 2)+trans)';
            r = robot.cap{i-1}.r;
            [x1, y1, z1] = cylinder(r, 10, c1, c2);
            handle(i-1, 1) = surf(x1, y1, z1, 'FaceColor',color,'EdgeColor','None');
            handle(i-1, 2) = surf(sx*r+c1(1), sy*r+c1(2), sz*r+c1(3), 'FaceColor',color,'EdgeColor','None');
            handle(i-1, 3) = surf(sx*r+c2(1), sy*r+c2(2), sz*r+c2(3), 'FaceColor',color,'EdgeColor','None');
            alpha(handle(i-1, 1),valpha);
            alpha(handle(i-1, 2),valpha);
            alpha(handle(i-1, 3),valpha);
        end
        if i == n
            rot = M{i}(1:3,1:3);
            trans = M{i}(1:3,4);
            for j = 1:robot.cap{i-1}.num
                eval(['p = robot.cap{i-1}.p' num2str(j) ';']);
                eval(['r = robot.cap{i-1}.r' num2str(j) ';']);
                c1 = (rot*p(:, 1)+trans)';
                c2 = (rot*p(:, 2)+trans)';
                [x1, y1, z1] = cylinder(r, 10, c1, c2);
                handle(i-1, 1) = surf(x1, y1, z1, 'FaceColor',color,'EdgeColor','None');
                handle(i-1, 2) = surf(sx*r+c1(1), sy*r+c1(2), sz*r+c1(3), 'FaceColor',color,'EdgeColor','None');
                handle(i-1, 3) = surf(sx*r+c2(1), sy*r+c2(2), sz*r+c2(3), 'FaceColor',color,'EdgeColor','None');
                alpha(handle(i-1, 1),valpha);
                alpha(handle(i-1, 2),valpha);
                alpha(handle(i-1, 3),valpha);
            end
        end
    end
    robot.handle = handle;
    

    hold on 
    plot3(arr(1,:),arr(2,:),arr(3,:),'*-','color','b','lineWidth',2);
    

    [laser, laser_rot] = ForKine(safe_theta(:,cnt), robot.DH, robot.base, robot.Msix2tool);
    hold on 
    plot3(laser(1),laser(2),laser(3),'*-','color','r','lineWidth',2)
    laser_rotM = axang2rotm([laser_rot / norm(laser_rot), norm(laser_rot)]);

    vecX = [laser, laser + 0.5*laser_rotM(:,1)];
    vecY = [laser, laser + 0.5*laser_rotM(:,2)];
    vecZ = [laser, laser + 0.5*laser_rotM(:,3)];
    hold on 
    plot3(vecX(1,:),vecX(2,:),vecX(3,:),'*-','color','r','lineWidth',2);
    hold on 
    plot3(vecY(1,:),vecY(2,:),vecY(3,:),'*-','color','g','lineWidth',2);
    hold on 
    plot3(vecZ(1,:),vecZ(2,:),vecZ(3,:),'*-','color','b','lineWidth',2);
    hold on;
    laser_rotM = M_target(:,(cnt-1)*3 + 1:cnt*3);
    vecX = [laser, laser + 0.5*laser_rotM(:,1)];
    vecY = [laser, laser + 0.5*laser_rotM(:,2)];
    vecZ = [laser, laser + 0.5*laser_rotM(:,3)];
    hold on 
    plot3(vecX(1,:),vecX(2,:),vecX(3,:),'*--','color','r','lineWidth',2);
    hold on 
    plot3(vecY(1,:),vecY(2,:),vecY(3,:),'*--','color','g','lineWidth',2);
    hold on 
    plot3(vecZ(1,:),vecZ(2,:),vecZ(3,:),'*--','color','b','lineWidth',2);
    hold on;

    

    xlim=[-1,2.5];
    ylim=[-0.5,0.5];
    zlim=[0,2];

    axis equal
    axis([xlim,ylim,zlim]);
    lighting=camlight('left');

    set(gca,'Color',[0.8 0.8 0.8]);
    wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[63,64,64]/255);

    wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(2),ylim(2),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[63,64,64]/255);
    zlabel('z axis');
    ylabel('y axis');
    xlabel('x axis');
    view(0,80);




    F(framen) = getframe(gcf);
    framen = framen + 1;



end

end