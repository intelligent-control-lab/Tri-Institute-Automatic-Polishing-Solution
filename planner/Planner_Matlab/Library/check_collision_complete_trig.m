
function col_flag = check_collision_complete_trig(theta0,robot,triangles)
col_flag = 0;
ntrig = size(triangles, 2);
distList = [];
parfor j = 1:ntrig
    triangle = cell2mat(triangles(j));
    distList(j) = dist_arm_triangle_complete(theta0, robot.DH, robot.base, robot.cap,triangle);
end
if min(distList) <= 0
    col_flag = 1;
end


    