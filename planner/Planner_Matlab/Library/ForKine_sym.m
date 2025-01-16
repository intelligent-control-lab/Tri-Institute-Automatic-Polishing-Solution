
function epos = ForKine_sym(x,DH,base, Msix2tool)






         
syms t1 t2     
nlink=size(DH,1);
DH(:,1) = x;
DHnew = [t1;t2];
DHnew = [DHnew DH(1:2,2:4)];
DH = [];
DH = DHnew;
M=cell(1,nlink+1); M{1}=eye(4);












for i=1:2
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
epos = M{i+1}(1:3,4)+base;
end