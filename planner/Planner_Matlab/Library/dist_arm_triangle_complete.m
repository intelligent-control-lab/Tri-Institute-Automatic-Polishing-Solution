
function d = dist_arm_triangle_complete(theta,DH,base,cap,triangle)
nlink = size(DH,1);
DH(:,1) = theta;
d = Inf;
[pos,~]=CapPos(base,DH,cap);
distList = [];
parfor i=1:size(pos,2)
    point1 = pos{i}.p(:,1);
    point2 = pos{i}.p(:,2);
    radius = pos{i}.r;
    p1 = triangle(:,1);
    p2 = triangle(:,2);
    p3 = triangle(:,3);
    [dist, flag, points] = distLinSeg2Tri(point1,point2,p1,p2,p3);
    dist = dist - radius;
    distList(i) = dist;
end
d = min(distList);
end
