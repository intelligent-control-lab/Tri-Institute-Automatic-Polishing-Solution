p1 = [0; 0; 0];
p2 = [5; 0; 0];
p3 = [0; 5; 0];

A = [3; 6; 3];
B = [10; -7; 6];

cnt = 1;
P1 = -10 + 20 * rand(3,cnt);
P2 = -10 + 20 * rand(3,cnt);
P3 = -10 + 20 * rand(3,cnt);
AList = -10 + 20 * rand(3,cnt);
BList = -10 + 20 * rand(3,cnt);
tic

for i=1:1:cnt
    A = AList(:,i);
    B = BList(:,i);
    p1 = P1(:,i);
    p2 = P2(:,i);
    p3 = P3(:,i);
   [dist, flag, points] = distLinSeg2Tri(A,B,p1,p2,p3);
end

T = toc
T_mean = T / cnt

planes = plane_of_triangle(p1, p2, p3);

Ap = proj_plane(A, planes);
flagA = checkPointInTri(Ap, p1, p2, p3);

Bp = proj_plane(B, planes);
flagB = checkPointInTri(Bp, p1, p2, p3);

pointIn = intersectLinSeg2Tri(A,B,p1,p2, p3);


AAp = [A, Ap];
BBp = [B, Bp];
AB = [A, B];
x1 = -10:0.05:10;
y1 = -10:0.05:10;
[x, y] = meshgrid(x1,y1);
z = (-planes(1)*x - planes(2)*y - planes(4)) / planes(3);

clf
color = [255,255,0]/255;
P = [p1,p2,p3,p1];
surf(x,y,z,'FaceColor',color, 'EdgeColor','None', "FaceAlpha",0.5);
hold on;
plot3(P(1, :),P(2, :),P(3, :),'bo-')
hold on;
plot3(AAp(1, :),AAp(2, :),AAp(3, :),'r*-')
hold on;
plot3(BBp(1, :),BBp(2, :),BBp(3, :),'r*-')
hold on;
plot3(AB(1, :),AB(2, :),AB(3, :),'b*-')
hold on;
if size(pointIn) ~= 0
    plot3(pointIn(1),pointIn(2),pointIn(3),'g^');
end
hold on;
points = points';
plot3(points(1, :),points(2, :),points(3, :),'g^-')
axis equal;
axis([-10, 10, -10, 10,-10, 10])
view(-30,10)