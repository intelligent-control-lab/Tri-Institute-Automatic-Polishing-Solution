function feasible = checkFeasible(theta, PC, PC_idx, DH, base, cap, Msix2tool)


PC_minX = min(PC(:,1));
PC_maxX = max(PC(:,1));
PC_minY = min(PC(:,2));
PC_maxY = max(PC(:,2));
PC_minZ = min(PC(:,3));
PC_maxZ = max(PC(:,3));
feasible = 1;

offset = 0.3;
PC_hull = PC(PC_idx < 0, :);


DH(:,1) = theta;
[pos,~]=CapPos(base,DH,cap, Msix2tool);
for i=1:size(pos,2)
    point1 = pos{i}.p(:,1);
    point2 = pos{i}.p(:,2);
    radius = pos{i}.r;
    if point1(1) > PC_minX + offset && inhull(point1', PC_hull) == 0
        feasible = 0;


        return;
    end
    if point2(1) > PC_minX + offset && inhull(point2', PC_hull) == 0
        feasible = 0;


        return;
    end
end















end

