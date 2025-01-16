function pointIn = intersectLinSeg2Tri(pointL1,pointL2,pointT1,pointT2, pointT3)
    plane = plane_of_triangle(pointT1, pointT2, pointT3);  
    d = pointL2 - pointL1;
    dx = d(1);
    dy = d(2);
    dz = d(3);
    A = plane(1);
    B = plane(2);
    C = plane(3);
    D = plane(4);

    pointIn = [];
    if A*dx + B*dy + C*dz == 0
        return;
    end

    t = -(pointL1(1)*A + pointL1(2)*B + pointL1(3)*C + D) / (A*dx + B*dy + C*dz);

    if t < 0 || t > 1
        return;
    end
    
    pointIn = pointL1 + t * d;

    if checkPointInTri(pointIn, pointT1, pointT2, pointT3) == 0
        pointIn = [];
        return;
    end

    

    return;
end
