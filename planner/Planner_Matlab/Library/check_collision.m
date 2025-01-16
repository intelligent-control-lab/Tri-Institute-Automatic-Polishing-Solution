
function no_col_flag = check_collision(theta0,robot,planes)
no_col_flag = 0;
nplanes = size(planes, 1);
for j = 1:nplanes
    plane = planes(j,:);
    dist = dist_arm_plane(theta0, robot.DH, robot.base, robot.cap, plane);

    if dist <= 0
        no_col_flag = 1;
        break
    end
end
