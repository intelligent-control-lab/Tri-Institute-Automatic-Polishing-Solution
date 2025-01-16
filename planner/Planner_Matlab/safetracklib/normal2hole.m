
function unitnorm = normal2hole(planes,point)
if nargin < 2
    point = [1.427, 0, 1.459];
end
if nargin == 2
    point = point';
end
unit2h = [];
for i = 1:size(planes,1)
    normal = planes(i,1:3);
    unitn = normal / norm(normal);
    d = planes(i,4);
    c = planes(i,3);
    O = [0,0,-d/c];
    OP = point - O;
    if dot(OP,normal) > 0
        unit2h = [unit2h;unitn];
    else
        unit2h = [unit2h;-unitn];
    end
end
unitnorm = unit2h(1,:);




















































