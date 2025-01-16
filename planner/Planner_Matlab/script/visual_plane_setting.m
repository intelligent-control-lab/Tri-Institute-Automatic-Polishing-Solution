
load('data/axis3Relative_wpframe.mat');
load('data/cross_segs_wpframe.mat');

theta = 0;      

base = [0;0;0];
M1 = [0.91647976 -0.00729021  0.40001462  1.40608102;
      0.02281488  0.99915928 -0.03406199 -0.02509452;
     -0.39943     0.0403434   0.91587558  1.04437673;
      0.          0.          0.          1.   ];

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