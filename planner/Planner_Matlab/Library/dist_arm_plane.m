
function d = dist_arm_plane(theta,DH,base,cap,plane,consider_line)









nlink = size(DH,1);
DH(:,1) = theta;
d = Inf;


[pos,~]=CapPos(base,DH,cap);


if nargin < 6
  consider_line = 1.2-cap{5}.r;
end
line_x = consider_line;
pos_csd = [];
r_csd = [];

for i = 1:nlink
    if pos{i}.p(1,1) < line_x && pos{i}.p(1,2) < line_x

        continue  
    end
    if pos{i}.p(1,1) < line_x && pos{i}.p(1,2) > line_x

        diff = pos{i}.p(:,2) - pos{i}.p(:,1);
        ratio = (line_x - pos{i}.p(1,1)) / (pos{i}.p(1,2) - pos{i}.p(1,1));
        pos_tail_1 = pos{i}.p(:,1) + ratio*diff;
        pos_tail = [pos_tail_1, pos{i}.p(:,2)];
        pos_csd = [pos_csd {pos_tail}];
        r_csd = [r_csd pos{i}.r];
        continue
    end
    if pos{i}.p(1,1) > line_x && pos{i}.p(1,2) > line_x

        pos_csd = [pos_csd {pos{i}.p}];
        r_csd = [r_csd pos{i}.r];
    end
end







for i=1:size(pos_csd,2)
    pos_tmp = cell2mat(pos_csd(i));
    point1 = pos_tmp(:,1);
    point2 = pos_tmp(:,2);

    proj1 = proj_plane(point1, plane);
    proj2 = proj_plane(point2, plane);

    

    n = normal2hole(plane);
    assert(norm(n)<1.001  && norm(n)>0.999,['normal is not unit vector, since n=' num2str(norm(n))]);

    vec1 = point1 - proj1;
    vec2 = point2 - proj2;
    dis1 = dot(n,vec1);
    dis2 = dot(n,vec2);
    if dis1 >= dis2 
        dis = dis2 - r_csd(i);
    else
        dis = dis1 - r_csd(i);
    end

    if dis < d
        d = dis;
    end
end
end