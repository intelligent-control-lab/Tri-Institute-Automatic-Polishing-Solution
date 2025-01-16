
function d = closest_dist_plane(theta0,robot,planes,LineSegs,anchor_point,consider_line)
    nplanes = size(planes, 1);
    min_d = 999999;
    for j = 1:nplanes
        plane = planes(j,:);
        lineseg = LineSegs{j}.p;
        dist = dist_arm_plane_complete(theta0, robot.DH, robot.base, robot.cap, plane, lineseg, anchor_point, consider_line);
        if min_d > dist 
            min_d = dist;
        end
    end
    d = min_d;
end