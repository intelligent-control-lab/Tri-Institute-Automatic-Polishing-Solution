function inside = checkInHull(point, PC)


PC_minX = min(PC(:,1));
PC_maxX = max(PC(:,1));
PC_minY = min(PC(:,2));
PC_maxY = max(PC(:,2));
PC_minZ = min(PC(:,3));
PC_maxZ = max(PC(:,3));
inside = 1;
if point(1) < PC_minX || point(1) > PC_maxX
    inside = 0;
    return;
end
if point(2) < PC_minY || point(2) > PC_maxY
    inside = 0;
    return;
end
if point(3) < PC_minZ || point(3) > PC_maxZ
    inside = 0;
    return;   
end
end

