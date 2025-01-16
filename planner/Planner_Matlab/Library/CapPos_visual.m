function M=CapPos_visual(robot)
DH = robot.DH;
Msix2tool = robot.Msix2tool;
nlink=size(DH,1);
M=cell(1,nlink+1); M{1}=eye(4); M{1}(1:3, 4) = robot.base;
for i=1:nlink
    if i <= nlink
        R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
            sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
            0  sin(DH(i,4)) cos(DH(i,4))];
        T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
        M{i+1}=M{i}*[R T; zeros(1,3) 1];
    end
    if i == nlink
        M{i+1}=M{i+1}*Msix2tool;
    end
end
end