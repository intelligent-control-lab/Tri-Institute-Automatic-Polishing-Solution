





load('data/axis3Relative_info_new.mat');
load('data/cross_segs_info_new.mat');


      










P1P2 = 0;
      

base = [wp2robot;0;height];


DH = [0, 0.7975, 0, pi/2;
      theta, 0, 0, -pi/2];

M=cell(1,2+1); M{1}=eye(4);
for i=1:2
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T; zeros(1,3) 1];
end  

axis3origin = M{3}(1:3,1:3)*[0;0;0] + M{3}(1:3,4) + base;


arr_rotate = M{3}(1:3,1:3)*arr_axis3 + M{3}(1:3,4) + base;


for t = 1:size(LineSegs,2)
    LineSegs{t}.p = M{3}(1:3,1:3)*LineSegs{t}.p + M{3}(1:3,4) + base;
end


point_anchor_rotate = M{3}(1:3,1:3)*point_anchor_axis3 + M{3}(1:3,4) + base;


abcR = [];

for i = 1:8
    normalP = M{3}(1:3,1:3)*abc(i,:)' + M{3}(1:3,4) + base;
    abcRow = normalP - axis3origin;
    abcR = [abcR; abcRow'];
end

dR = [];
for i = 1:8
    point = M{3}(1:3,1:3)*planePoints(i,:)' + M{3}(1:3,4) + base;
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


P1P2 = 0.513 * cos(-theta);
OP1 = 0.513 * sin(-theta);
xp3 = axis3origin(1) + 0.1275*sin(-theta);
zp3 = axis3origin(3) + 0.1275*cos(-theta);





















