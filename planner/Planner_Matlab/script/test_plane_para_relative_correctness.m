clc
clear 
load('data/axis3Relative_wpframe.mat');
load('data/cross_segs_wpframe.mat');
theta = 0;      
base = [0;0;0];
M1 =  [0.81511451, -0.07717321, 0.57413642, 1.25674456;
      0.13794308, 0.9884358, -0.06297919, -0.03413396;
      -0.56263668, 0.1305334, 0.81633387, 0.984576;
      0, 0, 0, 1];
M = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];
axis3origin = M(1:3,1:3)*[0;0;0] + M(1:3,4) + base;
arr_rotate = M(1:3,1:3)*arr_axis3 + M(1:3,4) + base;
for t = 1:size(LineSegs,2)
    LineSegs{t}.p = M(1:3,1:3)*LineSegs{t}.p + M(1:3,4) + base;
end
point_anchor_rotate = M(1:3,1:3)*point_anchor_axis3 + M(1:3,4) + base;
abcR = [];
for i = 1:8
    normalP = M(1:3,1:3)*abc(i,:)' + M(1:3,4) + base;
    abcRow = normalP - axis3origin;
    abcR = [abcR; abcRow'];
end

dR = [];
for i = 1:8
    point = M(1:3,1:3)*planePoints(i,:)' + M(1:3,4) + base;
    dRow = -dot(point',abcR(i,:));
    dR = [dR;dRow];
end

planes_new = [abcR dR];
planes = planes_new;

for i = 1:8
    if (i ~= 3 && i ~= 7)
        planes(i,:) = planes(i,:)/(-planes(i,3));
    else
        planes(i,:) = planes(i,:)/(-planes(i,2));
    end
end
arr = arr_rotate;
point_anchor_rotate = M1(1:3,1:3)*point_anchor_rotate + M1(1:3,4);
plot3(point_anchor_rotate(1,:),point_anchor_rotate(2,:),point_anchor_rotate(3,:),'*-','color','r');
hold on 
arr = M1(1:3,1:3)*arr + M1(1:3,4) + base;
plot3(arr(1,:),arr(2,:),arr(3,:),'*-','color','b');
hold on 
valpha = 0.8;
color = [255,255,243]/255;
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
    handles(i)=surf(x,y,z,'FaceColor',color,'EdgeColor','None');
    alpha(handles(i),valpha);
    hold on 
end
for i = 1:size(LineSegs,2)
    p = LineSegs{i}.p;
    p = M1(1:3,1:3)*p + M1(1:3,4);
    plot3(p(1,:),p(2,:),p(3,:),'*-','color','r');
    hold on
end
axis equal
lighting=camlight('left');
xlim([0 2.5]);
ylim([-0.5 0.5]);
zlim([0 2]);
set(gca,'Color',[0.8 0.8 0.8]);
zlabel('z axis');
ylabel('y axis');
xlabel('x axis');
view(60,0);