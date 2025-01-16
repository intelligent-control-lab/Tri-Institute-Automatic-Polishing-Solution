function [dist,flag points] = distLinSeg2Tri(pointL1,pointL2,pointT1,pointT2, pointT3)

    [distL1, flagL1, pointsL1] = distPnt2Tri(pointL1, pointT1,pointT2, pointT3);
    [distL2, flagL2, pointsL2] = distPnt2Tri(pointL2, pointT1,pointT2, pointT3);
    flag = [flagL1, flagL2];
    pointIn = intersectLinSeg2Tri(pointL1,pointL2,pointT1,pointT2, pointT3);
    
    if size(pointIn) ~= 0
        dist = 0;
        points = [pointIn'; pointIn'];
        return;
    end

    [d1, points1] = distLinSeg(pointL1',pointL2',pointT1',pointT2');
    [d2, points2] = distLinSeg(pointL1',pointL2',pointT2',pointT3');
    [d3, points3] = distLinSeg(pointL1',pointL2',pointT3',pointT1');

    [dist, index] = min([distL1, distL2, d1, d2, d3]);
    pointsList = {pointsL1, pointsL2, points1, points2, points3};
    points = cell2mat(pointsList(index));
end
