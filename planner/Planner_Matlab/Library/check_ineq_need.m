
function need_flag = check_ineq_need(theta,DH,base,cap, Msix2tool,consider_line)
nlink = size(DH,1);
DH(:,1) = theta;
d = Inf;


[pos,~]=CapPos(base,DH,cap,Msix2tool);


if nargin < 6

  consider_line = 1.2-cap{5}.r;
end
line_x = consider_line;
pos_csd = [];
r_csd = [];
cross_line_num = 0;

for i = 1:nlink
    if pos{i}.p(1,1) < line_x && pos{i}.p(1,2) < line_x

        continue  
    else
        cross_line_num = cross_line_num + 1;
    end
end


if cross_line_num == 0
    need_flag = 0;
else
    need_flag = 1;
end

