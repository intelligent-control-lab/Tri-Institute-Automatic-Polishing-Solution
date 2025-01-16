function visualize_traj(safe_theta,arr,planes,wp2robot,height,theta,xp3,zp3,P1P2,OP1)
robot=robotproperty('GP50');




robotCAD = load(strcat('figure/',robot.name,'.mat'));
scale = 1;

framen = 1;


xs = [];
ys = [];
zs = [];
for i = 1:size(planes,1)
    if planes(i,3) == -1


        x1 = xp3-P1P2-0.5:0.05:xp3+P1P2;
        y1 = -0.205:0.05:0.205;
        [x, y] = meshgrid(x1,y1);
        z = planes(i,1)*x + planes(i,2)*y + planes(i,4);
    end
    if planes(i,2) == -1


        x1 = xp3-P1P2-0.5:0.05:xp3+P1P2;
        z1 = zp3-OP1:0.02:zp3+OP1+0.4;
        [x, z] = meshgrid(x1,z1);
        y = planes(i,1)*x + planes(i,3)*z + planes(i,4);
    end
    xs = [xs {x}];
    ys = [ys {y}];
    zs = [zs {z}];
end


base = [wp2robot;0;height];


DH = [0, 0.7975, 0, pi/2;
      theta, 0, 0, -pi/2];

M1=cell(1,2+1); M1{1}=eye(4); M1{1}(1:3, 4) = base;
for i=1:2
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M1{i+1}=M1{i}*[R T; zeros(1,3) 1];
end  



for cnt = 58:58
    robot.DH(:,1) = safe_theta(:,cnt);
    M=CapPos_visual(robot);

    hold on;

    handle=[];

    for i=1:numel(robotCAD.workpiece)
        f=robotCAD.workpiece{i}.f; v=robotCAD.workpiece{i}.v.*scale; c=robotCAD.workpiece{i}.c; 
        color=[237,229,116]/255;




        Mtmp = [eye(3,3) [-0.513,0,0.1275]';
                zeros(1,3) 1];
        Mnew = M1{3} * Mtmp;
        v = setVertice(v,Mnew); 
        handle.workpiece(i) = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None', 'FaceAlpha',.3);
    end
























































































    





























end
















end