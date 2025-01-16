



clear
clc;

load('data/weld.mat')

planes = [-0.13677271909286323, -0.001307522919873557, -1, 0.35681409561655236;
          -0.09944386985869111, 0.9507137174883696, -1, 0.4148449626591891;
          -0.0028754998304916695, -1, -0.00048562669056519125, -0.1531872239587;
          -0.028656068479931213, -0.9169732738103323, -1, -0.02635994915558708;
          -0.04227291092492677, -0.0017900871745648592, -1, 0.04253270252138967;
          -0.027539864123062554, 0.9174694697754313, -1, -0.02670323898692613;
          0.003517293221011959, -1, -0.0026534115250074763, 0.1535545078421211;
          -0.09670160529976317, -0.9547657737231579, -1, 0.4150487079383617];
      
point_anchor = [0.2137; 0; 0.1701];
arr = weld;
theta = 0;
axis3origin = [0,0,0];

consider_line = 0;
LineSegs = planes_cross_property(planes,consider_line);


base = [0;0;0];

M = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];
M1 =  [0.81511451, -0.07717321, 0.57413642, 1.25674456;
      0.13794308, 0.9884358, -0.06297919, -0.03413396;
      -0.56263668, 0.1305334, 0.81633387, 0.984576;
      0, 0, 0, 1];



point_anchor = M1(1:3,1:3)*point_anchor + M1(1:3,4);
plot3(point_anchor(1,:),point_anchor(2,:),point_anchor(3,:),'*-','color','r');
hold on 


arr = M1(1:3,1:3)*arr' + M1(1:3,4) + base;
plot3(arr(1,:),arr(2,:),arr(3,:),'*-','color','b');
hold on 


valpha = 0.8;
color = [255,255,243]/255;

for i = 1:size(planes,1)
    if planes(i,3) == -1
        x1 = 0:0.05:1.026;
        y1 = -0.205:0.05:0.205;
        [x, y] = meshgrid(x1,y1);
        z = planes(i,1)*x + planes(i,2)*y + planes(i,4);
    end
    if planes(i,2) == -1
        x1 = 0:0.05:1.026;
        z1 = 0:0.02:0.3;
        [x, z] = meshgrid(x1,z1);
        y = planes(i,1)*x + planes(i,3)*z + planes(i,4);
    end   

    [x,y,z] = rotatePlane(x,y,z,M1);
    handles(i)=surf(x,y,z,'FaceColor',color,'EdgeColor','None');
    alpha(handles(i),valpha);
    hold on 
end


for i = 1:size(LineSegs,2)
    p = LineSegs{i}.p;
    p = M1(1:3,1:3)*p + M1(1:3,4);
    plot3(p(1,:),p(2,:),p(3,:),'*-','color','r');
    hold on
end



axis equal
lighting=camlight('left');
xlim([0 2.5]);
ylim([-0.5 0.5]);
zlim([0 2]);




set(gca,'Color',[0.8 0.8 0.8]);



zlabel('z axis');
ylabel('y axis');
xlabel('x axis');
view(60,0);