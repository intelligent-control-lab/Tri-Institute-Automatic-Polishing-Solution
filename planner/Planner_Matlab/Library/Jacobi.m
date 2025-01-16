








function J=Jacobi(theta,DH,n,p)
DH(:,1) = theta;
z=nan(3,n);
r_0=nan(3,n+1);
J=nan(6,n);
TCP_T=eye(4);

JointJacobi={};

for i=1:n
    JointJacobi{i}=nan(3,i);
end

alpha=DH(:,4);A=DH(:,3);D=DH(:,2);q=DH(:,1);

for i=1:n
    z(:,i)=TCP_T(1:3,3);
    r_0(:,i)=TCP_T(1:3,4);
    TCP_T=TCP_T*...
        [cos(q(i)) -sin(q(i))*cos(alpha(i))  sin(q(i))*sin(alpha(i)) A(i)*cos(q(i));...
         sin(q(i))  cos(q(i))*cos(alpha(i)) -cos(q(i))*sin(alpha(i)) A(i)*sin(q(i));...
          0            sin(alpha(i))                cos(alpha(i))            D(i);...
          0                0                       0               1];
end
r_0(:,n+1)=TCP_T(1:3,4);


    
for i=1:n
    J(:,i)=[cross(r_0(:,i)-p,z(:,i));z(:,i)];
    for j=i:n
        JointJacobi{j}(:,i)=cross(r_0(:,i)-r_0(:,j+1),z(:,i));
    end
end