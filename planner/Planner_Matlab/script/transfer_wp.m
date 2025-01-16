load('data/traj.mat')

planes = [-0.8553854681017643, -0.0005463239779652795, -1, 2.9257986256071304;
          0.05748143290163125, -1, 0.06252308341911939, -0.3026991195122747;
          -0.7329844887892633, 0.017746299409831523, -1, 2.462147522592861;
          -0.10475571509951867, -1, -0.13116945145812048, 0.49306524145584574];
      
point_anchor = [1.427; 0; 1.459];
      

cos_theta = 0.135 / sqrt(0.135^2 + 0.081^2);
theta = acos(cos_theta);
OP1 = 0.125/cos_theta;
P1P2 = 0.125*tan(theta);
P1 = [1.2;-0.175;0.8 + OP1];
P1_origin = [1.2;-0.175;1.468];
decrease_vec = P1_origin - P1;
decrease = decrease_vec(3);

for i = 1:size(arr,1)
    arr(i,:) = arr(i,:) - [0,0,decrease];
end

point_anchor = point_anchor - [0;0;decrease];

for i = 1:4
    plane = planes(i,:);
    c = plane(3);
    d = plane(4);
    dnew = decrease*c + d;
    planes(i,4) = dnew; 
end


base = [1.2;0;0];
DH = [0, 0.8, 0, pi/2;
      0, 0, 0, -pi/2];

M=cell(1,2+1); M{1}=eye(4);
for i=1:2
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T; zeros(1,3) 1];
end  
axis3origin = M{3}(1:3,1:3)*[0;0;0] + M{3}(1:3,4) + base;
arr = arr';
arr_axis3 = arr - axis3origin;
point_anchor_axis3 = point_anchor - axis3origin;
abc = planes(:,1:3);
planePoints = [];
for i = 1:4
    a = planes(i,1);
    b = planes(i,2);
    c = planes(i,3);
    d = planes(i,4);
    x3 = axis3origin(1);
    z3 = axis3origin(3);
    zaxis3 = (-d - x3*a)/c - z3;
    planePoints = [planePoints; [0,0,zaxis3]];
end

save('data/axis3Relative_info.mat','abc','planePoints','point_anchor_axis3','arr_axis3')


base = [1.2;0;0];
DH = [0, 0.8, 0, pi/2;
      theta, 0, 0, -pi/2];

M=cell(1,2+1); M{1}=eye(4);
for i=1:2
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T; zeros(1,3) 1];
end  

arr_rotate = M{3}(1:3,1:3)*arr_axis3 + M{3}(1:3,4) + base;


point_anchor_rotate = M{3}(1:3,1:3)*point_anchor_axis3 + M{3}(1:3,4) + base;


abcR = [];

for i = 1:4
    normalP = M{3}(1:3,1:3)*abc(i,:)' + M{3}(1:3,4) + base;
    abcRow = normalP - axis3origin;
    abcR = [abcR; abcRow'];
end

dR = [];
for i = 1:4
    point = M{3}(1:3,1:3)*planePoints(i,:)' + M{3}(1:3,4) + base;
    dRow = -dot(point',abcR(i,:));
    dR = [dR;dRow];
end

planes_new = [abcR dR];
planes_old = planes;
planes = planes_new;

for i = 1:2:3
    planes(i,:) = planes(i,:)/(-planes(i,3));
end
for i = 2:2:4
    planes(i,:) = planes(i,:)/(-planes(i,2));
end

arr = arr_rotate;





















