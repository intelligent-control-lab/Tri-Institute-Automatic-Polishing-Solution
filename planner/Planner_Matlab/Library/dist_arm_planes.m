function dist_tmp = dist_arm_planes(theta0, robot, planes, LineSegs, anchor_point, consider_line)

dist_tmp = inf;
nplanes = size(planes, 1);
for j = 1:nplanes
    plane = planes(j,:);
    lineseg = LineSegs{j}.p;
    dist = dist_arm_plane_complete(theta0, robot.DH, robot.base, robot.cap, plane, lineseg, anchor_point, consider_line);
    if dist <= dist_tmp
        dist_tmp = dist;
    end
end
end
