

plot3(point_anchor_rotate(1,:),point_anchor_rotate(2,:),point_anchor_rotate(3,:),'*-','color','r');
hold on 


plot3(arr(1,:),arr(2,:),arr(3,:),'*-','color','b');
hold on 


valpha = 0.8;
color = [255,255,243]/255;

for i = 1:size(planes,1)
    if planes(i,3) == -1

        x1 = xp3-P1P2:0.05:xp3+P1P2;
        y1 = -0.205:0.05:0.205;
        [x, y] = meshgrid(x1,y1);
        z = planes(i,1)*x + planes(i,2)*y + planes(i,4);
    end
    if planes(i,2) == -1


        x1 = xp3-P1P2:0.05:xp3+P1P2;
        z1 = zp3-OP1:0.02:zp3+OP1+0.3;
        [x, z] = meshgrid(x1,z1);
        y = planes(i,1)*x + planes(i,3)*z + planes(i,4);
    end   
    handles(i)=surf(x,y,z,'FaceColor',color,'EdgeColor','None');
    alpha(handles(i),valpha);
    hold on 
end


for i = 1:size(LineSegs,2)
    p = LineSegs{i}.p;
    plot3(p(1,:),p(2,:),p(3,:),'*-','color','r');
    hold on
end































































axis equal
lighting=camlight('left');
xlim=[1,2.5];
ylim=[-0.5,0.5];
zlim=[0,4];

set(gca,'Color',[0.8 0.8 0.8]);



zlabel('z axis');
ylabel('y axis');
xlabel('x axis');
view(60,0);