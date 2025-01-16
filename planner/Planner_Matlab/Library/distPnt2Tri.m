function [dist, flag, points] = distPnt2Tri(pointP,pointT1,pointT2, pointT3)
    
    plane = plane_of_triangle(pointT1, pointT2, pointT3);  
    proj = proj_plane(pointP, plane);
    
    flag = checkPointInTri(proj, pointT1, pointT2, pointT3);

    if  flag == 1
        dist = norm(pointP - proj);
        points = [pointP'; proj'];
        return;
    end

    [d1, points1] = distLinSeg(pointP',pointP',pointT1',pointT2');
    [d2, points2] = distLinSeg(pointP',pointP',pointT2',pointT3');
    [d3, points3] = distLinSeg(pointP',pointP',pointT3',pointT1');
    [dist, index] = min([d1, d2, d3]);
    pointsList = {points1, points2, points3};
    points = cell2mat(pointsList(index));
    return;
end
