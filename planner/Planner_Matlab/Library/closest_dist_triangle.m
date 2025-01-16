
function d = closest_dist_triangle(theta0,robot,triangles)
    ntrig = size(triangles, 2);
    distList = [];
    parfor j = 1:ntrig
        triangle = cell2mat(triangles(j));
        dist = dist_arm_triangle_complete(theta0, robot.DH, robot.base, robot.cap,triangle);
        distList(j) = dist;
    end
    d = min(distList);
end