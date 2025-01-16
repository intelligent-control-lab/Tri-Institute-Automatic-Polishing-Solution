
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
[pos,M]=CapPos(robot.base,robot.DH,robot.cap);

wp2robot = 1.6;
height = -0.051;
for theta = -pi/2:pi/20:0

    clf;
    transfer_wp_new;
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

    robotCAD = load(strcat('figure/',robot.name,'.mat'));
    scale = 1;

    for i=1:numel(robotCAD.workpiece)
        f=robotCAD.workpiece{i}.f; v=robotCAD.workpiece{i}.v.*scale; c=robotCAD.workpiece{i}.c; 
        color=[237,229,116]/255;
        for j=1:size(v,1) 
            v(j,:) = v(j,:) - [0.513,0,-0.1275];
        end
        v = setVertice(v,M1{3});
        handle.workpiece(i) = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None', 'FaceAlpha',.3);
    end

    hold on 
    axis3origin = M{3}(1:3,1:3)*[0;0;0] + M{3}(1:3,4) + base;
    P2 = M{3}(1:3,1:3)*[0;0;0.1275] + M{3}(1:3,4) + base;
    vec = [axis3origin, P2];
    plot3(vec(1,:),vec(2,:),vec(3,:),'color','r','LineWidth',2);
    axis equal
    lighting=camlight('left');

    set(gca,'Color',[0.8 0.8 0.8]);

    zlabel('z axis');
    ylabel('y axis');
    xlabel('x axis');
    axis([0 3 -0.3 0.3 0 1.5]);
    view(0,0);
    F = getframe(gcf);
end