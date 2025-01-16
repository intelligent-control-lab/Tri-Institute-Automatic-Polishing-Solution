
function d = dist_arm_PC(theta,DH, base, cap, PC, Msix2tool, check)
nlink = size(DH,1);
DH(:,1) = theta;
d = Inf;
PC = PC';
if nargin < 7
  check = 0;
end
[pos,~]=CapPos(base,DH,cap, Msix2tool);
dmin = 0.02;
distList = zeros(size(pos,2), 1);
for i=1:size(pos,2)
    point1 = pos{i}.p(:,1);
    point2 = pos{i}.p(:,2);
    radius = pos{i}.r;
    dist = distLinSeg2PC(point1,point2,PC);
    dist = dist - radius - dmin;
    if dist < 0 && check == 1
        d = dist;
        return;
    end
    distList(i) = dist;
end
d = min(distList);
end
