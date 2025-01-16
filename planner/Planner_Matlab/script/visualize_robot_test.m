clc
clear

robot=robotproperty('GP50');
load('weld.mat');
weld = weld';
point_anchor = [0.2137; 0; 0.1701];
robotCAD = load(strcat('figure/',robot.name,'.mat'));
scale = 1;
modify = [0;0;0;0;0;0];
theta_ini = robot.DH(:,1) + modify;
robot.DH(:,1) = robot.DH(:,1) + modify;
M=CapPos_visual(robot);
clf;
hold on;
    handle = [];
valpha = 0.5;
color = [255,255,243]/255;
load(strcat('figure/', robot.name, 'Capsules_newtool.mat'));
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
































































axis equal
lighting=camlight('left');

set(gca,'Color',[0.8 0.8 0.8]);



zlabel('z axis');
ylabel('y axis');
xlabel('x axis');
view(60,0);

