function sketch_traj(safe_theta,arr,planes,P1P2,wp2robot)
robot=robotproperty('GP50');





robotCAD = load(strcat('figure/',robot.name,'.mat'));
scale = 1;



    figure(1); hold on
    title('Distance=2.1 m, Height=0 m, Theta=0 rad');
    xlim=[-1,wp2robot-P1P2+1.3];
    ylim=[-0.5,0.5];
    zlim=[0,2.6];
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
    view(25,10);
    

    hold on 
    plot3(arr(1,:),arr(2,:),arr(3,:),'*-','color','blue');
    
    






xs = [];
ys = [];
zs = [];
for i = 1:size(planes,1)
    if planes(i,3) == -1

        x1 = wp2robot-P1P2:0.1:wp2robot-P1P2+1;
        y1 = -0.205:0.05:0.205;
        [x, y] = meshgrid(x1,y1);
        z = planes(i,1)*x + planes(i,2)*y + planes(i,4);
    end
    if planes(i,2) == -1

        x1 = wp2robot-P1P2:0.1:wp2robot-P1P2+1;

        z1 = 0.3:0.02:1.5;
        [x, z] = meshgrid(x1,z1);
        y = planes(i,1)*x + planes(i,3)*z + planes(i,4);
    end
    xs = [xs {x}];
    ys = [ys {y}];
    zs = [zs {z}];
end


    hold on
    valpha = 0.6;
    color = [255,255,243]/255;


    for i = 1:size(planes,1)  

        x = cell2mat(xs(i));
        y = cell2mat(ys(i));
        z = cell2mat(zs(i));
        handles(i)=surf(x,y,z,'FaceColor',color,'EdgeColor','None');
        alpha(handles(i),valpha);
        hold on 
    end

frm_num = 1;

for cnt = 1:10:size(safe_theta,2)
    valpha= cnt/size(safe_theta,2);

    robot.DH(:,1) = safe_theta(:,cnt);
    M=CapPos_visual(robot);



    handle=[];






















    for i=1:5
        v=robotCAD.link{i}.v.*scale; f=robotCAD.link{i}.f; c=robotCAD.link{i}.c; 

        color=[249,212,35]/255;
        v = setVertice(v,M{i+1});



        gcf;
        h = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
        alpha(h,valpha);
    end



    valpha = 0.5;
    color = [255,255,243]/255;
    load(strcat('figure/', robot.name, 'Capsules.mat'));
    boundary = RoBoundary;
    handle=[];
    n=min([size(M,2), length(boundary)]);
    for i=1:n
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




    

    




















    F(frm_num) = getframe(gcf);
    frm_num = frm_num + 1;

end
















end