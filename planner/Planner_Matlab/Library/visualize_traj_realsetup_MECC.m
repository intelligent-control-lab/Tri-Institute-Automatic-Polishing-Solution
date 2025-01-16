function visualize_traj_realsetup_MECC(safe_theta, arr0, wp_pos)
robot=robotproperty('GP50');
robotCAD = load(strcat('figure/',robot.name,'.mat'));
scale = 1;
framen = 1;
[PC_origin, PC_idx] = load_PC(1);
[~, M_PC_0] = processPC(PC_origin, wp_pos(:,1));
arr0 = setVertice(arr0', M_PC_0^(-1))';

for cnt = 1:20:(size(safe_theta,2) + 20)
    
    cnt = min(cnt,size(safe_theta,2));
    disp(cnt);
    robot.DH(:,1) = safe_theta(:,cnt);
    
    M=CapPos_visual(robot);

    hold on;

    

    if nargin >= 3
        [PC, M1, base_point, center_point] = processPC(PC_origin, wp_pos(:, cnt));
    else
        [PC, M1, base_point, center_point] = processPC(PC_origin);
    end
    
    

    
    hold on;






    arr = setVertice(arr0', M1)';


































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
    hold on;

    laser = ForKine(safe_theta(:,cnt), robot.DH, robot.base, robot.cap);
    hold on 
    plot3(laser(1),laser(2),laser(3),'*-','color','r','lineWidth',2)

   
    hold on;
    if cnt > 1
        continue
    end
    hold on;
    handle=[];
    for i=1:numel(robotCAD.workpiece)
        f=robotCAD.workpiece{i}.f; v=robotCAD.workpiece{i}.v.*scale; c=robotCAD.workpiece{i}.c; 
        color=[237,229,116]/255;
        v = setVertice(v,M1);
        handle.workpiece(i) = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None', 'FaceAlpha',.3);
    end
    

    hold on 
    plot3(arr(1,:),arr(2,:),arr(3,:),'*-','color','b','lineWidth',2);
    
    

    

    xlim=[-1,2.5];
    ylim=[-0.5,0.5];
    zlim=[0,2];
    view([1,-0.5,0.4]);
    axis equal
    axis([xlim,ylim,zlim]);
    lighting=camlight('left');

    set(gcf,'unit','centimeters','position',[3,5,20,12]);
    set(gca,'Color',[0.8 0.8 0.8],'position',[0.01,0.01,0.99,0.99] );
    
    wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[63,64,64]/255);

    wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(2),ylim(2),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[63,64,64]/255);



    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    set(gca,'ztick',[]);
    view(-15,3);

    F(framen) = getframe(gcf);
    framen = framen + 1;



end

end