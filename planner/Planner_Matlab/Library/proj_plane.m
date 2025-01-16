
function proj = proj_plane(point, plane)
xi = point(1);
yi = point(2);
zi = point(3);
A = plane(1);
B = plane(2);
C = plane(3);
D = plane(4);
t = (A*xi + B*yi + C*zi + D) / (A^2 + B^2 + C^2);
proj = point;
proj(1) = xi - A*t;
proj(2) = yi - B*t;
proj(3) = zi - C*t;
end