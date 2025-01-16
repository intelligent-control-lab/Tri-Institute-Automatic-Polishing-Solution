function visualize_traj_realsetup(safe_theta,arr, arr2)
robot=robotproperty('GP50');
robotCAD = load(strcat('figure/',robot.name,'.mat'));
scale = 1;
framen = 1;


arr_record = arr;
xs = [];
ys = [];
zs = [];
visual_plane_setting;
for i = 1:size(planes,1)
    if planes(i,3) == -1
        x1 = 0:0.05:1.026;

        y1 = -0.205:0.05:0.205;
        [x, y] = meshgrid(x1,y1);
        z = planes(i,1)*x + planes(i,2)*y + planes(i,4);
    end
    if planes(i,2) == -1
        x1 = 0:0.05:1.026;

        z1 = 0:0.02:0.3;

        [x, z] = meshgrid(x1,z1);
        y = planes(i,1)*x + planes(i,3)*z + planes(i,4);
    end   

    [x,y,z] = rotatePlane(x,y,z,M1);
    xs = [xs {x}];
    ys = [ys {y}];
    zs = [zs {z}];
end

arr = arr_record;


for cnt = 1:size(safe_theta,2)

    robot.DH(:,1) = safe_theta(:,cnt);
    M=CapPos_visual(robot);
    clf;
    hold on;

    


    handle=[];

    for i=1:numel(robotCAD.workpiece)
        f=robotCAD.workpiece{i}.f; v=robotCAD.workpiece{i}.v.*scale; c=robotCAD.workpiece{i}.c; 
        color=[237,229,116]/255;
        v = setVertice(v,M1);
        handle.workpiece(i) = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None', 'FaceAlpha',.3);
    end
    
    



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

    [pos,M_0]=CapPos(robot.base,robot.DH,robot.cap);
    color = [rand rand rand];
    for i=6:8
        point1 = pos{i}.p(:,1);
        point2 = pos{i}.p(:,2);
        p = [point1, point2];
        plot3(p(1,:), p(2,:), p(3,:), '*-', 'Color',color);
        hold on;
    end


    valpha = 0.5;
    color = [255,255,243]/255;
    load(strcat('figure/', robot.name, 'Capsules_newtool2.mat'));
    boundary = RoBoundary;
    handle=[];
    n=min([size(M,2)-1, length(boundary)]);
    for i=1:n
        if i < n
            if isfield(boundary{i}, "X")
                X=boundary{i}.X;
                Y=boundary{i}.Y;
                Z=boundary{i}.Z;
                kd=size(X,1);jd=size(X,2);
                for k=1:kd
                    for j=1:jd
                        newvec=[X(k,j),Y(k,j),Z(k,j)]*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)';
                        X(k,j)=newvec(1);
                        Y(k,j)=newvec(2);
                        Z(k,j)=newvec(3);
                    end
                end
                handle(i)=surf(X,Y,Z,'FaceColor',color,'EdgeColor','None');

                alpha(handle(i),valpha);
            end
        end
        if i == n
            for p = 1:3
                if isfield(boundary{i+p-1}, "X")
                    X=boundary{i+p-1}.X;
                    Y=boundary{i+p-1}.Y;
                    Z=boundary{i+p-1}.Z;
                    kd=size(X,1);jd=size(X,2);
                    for k=1:kd
                        for j=1:jd
                            newvec=[X(k,j),Y(k,j),Z(k,j)]*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)';
                            X(k,j)=newvec(1);
                            Y(k,j)=newvec(2);
                            Z(k,j)=newvec(3);
                        end
                    end
                    handle(i+p-1)=surf(X,Y,Z,'FaceColor',color,'EdgeColor','None');

                    alpha(handle(i+p-1),valpha);
                end
            end
        end
    end























    

    hold on 
    plot3(arr(1,:),arr(2,:),arr(3,:),'*-','color','b','lineWidth',2);
    if nargin == 3
        hold on 
        plot3(arr2(1,:),arr2(2,:),arr2(3,:),'*-','color','r','lineWidth',2);
    end
    

    laser = ForKine(safe_theta(:,cnt), robot.DH, robot.base, robot.cap);
    hold on 
    plot3(laser(1),laser(2),laser(3),'*-','color','r','lineWidth',2)


    xlim=[-1,2.5];
    ylim=[-0.5,0.5];
    zlim=[0,2];
    view([1,-0.5,0.4]);
    axis equal
    axis([xlim,ylim,zlim]);
    lighting=camlight('left');

    set(gca,'Color',[0.8 0.8 0.8]);
    wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[63,64,64]/255);

    wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(2),ylim(2),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[63,64,64]/255);
    zlabel('z axis');
    ylabel('y axis');
    xlabel('x axis');

    view(-20,3);



    F(framen) = getframe(gcf);
    framen = framen + 1;

end

















TR = stlread('figure/wp_inSurf200.STL')













end