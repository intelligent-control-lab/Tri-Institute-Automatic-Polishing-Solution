
function pos = cross_point(p1,p2,p3,p4)

    x1 = p1(1);
    y1 = p1(2);
    z1 = p1(3);
    x2 = p2(1);
    y2 = p2(2);
    z2 = p2(3);
    x3 = p3(1);
    y3 = p3(2);
    z3 = p3(3);
    x4 = p4(1);
    y4 = p4(2);
    z4 = p4(3);
    

    dp1 = (p1 - p2);
    dp2 = (p3 - p4);
    dx1 = dp1(1);
    dy1 = dp1(2);
    dz1 = dp1(3);
    dx2 = dp2(1);
    dy2 = dp2(2);
    dz2 = dp2(3);
    

    x = (dx1*dx2*y3 - dx2*dx1*y1 - dx1*dy2*x3 + dx2*dy1*x1) / (dx2*dy1 - dx1*dy2);
    y = (dy1*(x - x1))/dx1 + y1;
    z = (dz1*(y - y1))/dy1 + z1;
    
    pos = [x;y;z];
end