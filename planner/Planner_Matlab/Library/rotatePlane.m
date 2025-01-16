
function [x1,y1,z1] = rotatePlane(x,y,z,M)
    assert(size(x,1) == size(y,1) && size(x,1) == size(z,1));
    assert(size(x,2) == size(y,2) && size(x,2) == size(z,2));
    x1 = zeros(size(x));
    y1 = zeros(size(y));
    z1 = zeros(size(z));
    for i = 1:size(x,1)
        for j = 1:size(x,2)
            axis = [x(i,j),y(i,j),z(i,j)];
            trans = M(1:3,1:3)*axis' + M(1:3,4);
            x1(i,j) = trans(1);
            y1(i,j) = trans(2);
            z1(i,j) = trans(3);
        end
    end
end