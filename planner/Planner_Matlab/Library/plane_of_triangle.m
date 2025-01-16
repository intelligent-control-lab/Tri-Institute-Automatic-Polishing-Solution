
function plane = plane_of_triangle(pointT1, pointT2, pointT3)
p1_x = pointT1(1);
p1_y = pointT1(2);
p1_z = pointT1(3);

p2_x = pointT2(1);
p2_y = pointT2(2);
p2_z = pointT2(3);

p3_x = pointT3(1);
p3_y = pointT3(2);
p3_z = pointT3(3);

A = ( (p2_y-p1_y)*(p3_z-p1_z)-(p2_z-p1_z)*(p3_y-p1_y) );
B = ( (p2_z-p1_z)*(p3_x-p1_x)-(p2_x-p1_x)*(p3_z-p1_z) );
C = ( (p2_x-p1_x)*(p3_y-p1_y)-(p2_y-p1_y)*(p3_x-p1_x) );
D = ( 0-(A*p1_x+B*p1_y+C*p1_z) );

plane(1) = A;
plane(2) = B;
plane(3) = C;
plane(4) = D;
end