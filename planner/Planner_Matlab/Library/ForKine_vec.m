
function target_vec = ForKine_vec(x, DH, base, vec, Msix2tool)

        






        







         
         






         
nlink=size(DH,1);
DH(:,1) = x;
M=cell(1,nlink+1); M{1}=eye(4);
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
epos = M{i+1}(1:3,4)+base;
rotate = rotm2axang(M{i+1}(1:3,1:3));
rotate_new = rotate(1:3) * rotate(4);

if vec == 'x' || vec == 'X'
    target_vec = M{i+1}(1:3,1);
end
if vec == 'y' || vec == 'Y'
    target_vec = M{i+1}(1:3,2);
end
if vec == 'z' || vec == 'Z'
    target_vec = M{i+1}(1:3,3);
end

end