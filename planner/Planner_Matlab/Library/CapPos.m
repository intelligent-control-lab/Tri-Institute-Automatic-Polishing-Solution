function [pos,M]=CapPos(base,DH,RoCap, Msix2tool)     
nlink=size(DH,1);
pos=cell(1,nlink+2);
M=cell(1,nlink+1); M{1}=eye(4);
for i=1:nlink
    if i < nlink
        R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
            sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
            0  sin(DH(i,4)) cos(DH(i,4))];
        T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
        M{i+1}=M{i}*[R T; zeros(1,3) 1];
        for k=1:2
            pos{i}.p(:,k)=M{i+1}(1:3,1:3)*RoCap{i}.p(:,k)+M{i+1}(1:3,4)+base;
        end
        pos{i}.r = RoCap{i}.r;
    end
    if i == nlink 
        R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
            sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
            0  sin(DH(i,4)) cos(DH(i,4))];
        T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
        M{i+1}=M{i}*[R T; zeros(1,3) 1];

        M{i+1}=M{i+1}*Msix2tool;

        for j = 1:RoCap{i}.num
            for k=1:2
                eval(['pos{i-1+j}.p(:,k)=M{i+1}(1:3,1:3)*RoCap{i}.p' num2str(j) '(:,k)+M{i+1}(1:3,4)+base;']);
            end
            eval(['pos{i-1+j}.r = RoCap{i}.r' num2str(j) ';']);
        end
    end
end
end