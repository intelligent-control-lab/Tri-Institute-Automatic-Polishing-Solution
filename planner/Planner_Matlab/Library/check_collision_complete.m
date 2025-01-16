
function col_flag = check_collision_complete(theta0,robot,planes,LineSegs,anchor_point,consider_line)
if nargin < 6
  consider_line = 1.2;
end
col_flag = 0;
nplanes = size(planes, 1);
for j = 1:nplanes
    plane = planes(j,:);
    lineseg = LineSegs{j}.p;
    dist = dist_arm_plane_complete(theta0, robot.DH, robot.base, robot.cap, plane, lineseg, anchor_point, consider_line);

    if dist <= 0
        col_flag = 1;
        break
    end
end
