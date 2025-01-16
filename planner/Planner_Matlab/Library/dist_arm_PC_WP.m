
function d = dist_arm_PC_WP(x,DH, base, cap, PC_origin, check)


theta = x(1:6,:);
wp_pos = [0;x(7);x(8)];
nlink = size(DH,1);
PC = processPC(PC_origin, wp_pos);
DH(:,1) = theta;
d = Inf;
PC = PC';
if nargin < 6
  check = 0;
end

[pos,~]=CapPos(base,DH,cap);


distList = zeros(size(pos,2), 1);
for i=1:size(pos,2)
    point1 = pos{i}.p(:,1);
    point2 = pos{i}.p(:,2);
    radius = pos{i}.r;
    

    dist = distLinSeg2PC(point1,point2,PC);
    dist = dist - radius;
    
    if dist < 0 && check == 1
        d = dist;
        return;
    end

    distList(i) = dist;
end
d = min(distList);


end
