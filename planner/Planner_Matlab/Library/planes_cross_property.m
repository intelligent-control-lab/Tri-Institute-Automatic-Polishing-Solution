function planeLineSegs = planes_cross_property(planes,cross_line)



if nargin < 2
    cross_line = 1.2;
end
nplanes = size(planes,1);
planeLineSegs = cell(1,nplanes);

for i = 1:nplanes
    pre = i;
    if i+1 > nplanes
        next = mod(i+1,nplanes);
    else
        next = i+1;
    end

    A1 = planes(pre,1);
    B1 = planes(pre,2);
    C1 = planes(pre,3);
    D1 = planes(pre,4);

    A2 = planes(next,1);
    B2 = planes(next,2);
    C2 = planes(next,3);
    D2 = planes(next,4);
    

    x = cross_line;
    y = -(D1*C2 - D2*C1 + x*A1*C2 - x*A2*C1) / (B1*C2 - B2*C1);
    z = -(D1*B2 - D2*B1 + x*A1*B2 - x*A2*B1) / (C1*B2 - C2*B1);
    

    planeLineSegs{pre}.p(:,2) = [x,y,z]';
    planeLineSegs{next}.p(:,1) = [x,y,z]';
end
end