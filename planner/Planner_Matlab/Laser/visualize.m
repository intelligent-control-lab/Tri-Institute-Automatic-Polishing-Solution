function visualize(safe_theta)
robot=robotproperty('GP50');
robotCAD = load(strcat('figure/',robot.name,'.mat'));
scale = 1;
framen = 1;

for cnt = 1:size(safe_theta,2)

    robot.DH(:,1) = safe_theta(:,cnt);
    
    M=CapPos_visual(robot);

    hold on;
    
    handle=[];


    for i=1:5
        v=robotCAD.link{i}.v.*scale; f=robotCAD.link{i}.f; c=robotCAD.link{i}.c; 

        color=[249,212,35]/255;
        v = setVertice(v,M{i+1});


        handle.link(i) = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
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
    

    laser = ForKine(safe_theta(:,cnt), robot.DH, robot.base, robot.cap);
    hold on 
    plot3(laser(1),laser(2),laser(3),'*-','color','r','lineWidth',2)

   
    hold on;

    

end
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


end