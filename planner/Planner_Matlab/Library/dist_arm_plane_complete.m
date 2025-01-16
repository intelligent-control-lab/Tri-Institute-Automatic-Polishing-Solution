
function d = dist_arm_plane_complete(theta,DH,base,cap,plane,lineseg,anchor_point,consider_line)








nlink = size(DH,1);
DH(:,1) = theta;
d = Inf;




[pos,~]=CapPos(base,DH,cap);



if nargin < 8
  consider_line = 1.2;
end

line_x = consider_line;






n = normal2hole(plane,anchor_point);
assert(norm(n)<1.001  && norm(n)>0.999,['normal is not unit vector, since n=' num2str(norm(n))]);




segp1 = lineseg(:,1);
segp2 = lineseg(:,2);

refp1 = [-0.5;0.5;2];

refproj = proj_plane(refp1, plane);
none_cross_norm = hatw(segp2 - segp1)*(refproj - segp1); 
none_cross_norm = none_cross_norm / norm(none_cross_norm);



for i=1:size(pos,2)
    point1 = pos{i}.p(:,1);
    point2 = pos{i}.p(:,2);
    radius = pos{i}.r;

    proj1 = proj_plane(point1, plane);
    proj2 = proj_plane(point2, plane);
    

    cross_norm1 = hatw(segp2 - segp1)*(proj1 - segp1);
    cross_norm2 = hatw(segp2 - segp1)*(proj2 - segp1);
    cross_norm1 = cross_norm1 / norm(cross_norm1);
    cross_norm2 = cross_norm2 / norm(cross_norm2);
    


    if i <= 8


        if (vecEq(cross_norm1,none_cross_norm)  && vecEq(cross_norm2,none_cross_norm))

            dist = distLinSeg(point1',point2',lineseg(:,1)',lineseg(:,2)') - radius;

        end



        if (vecEq(cross_norm1,none_cross_norm) && vecNeq(cross_norm2,none_cross_norm)) || (vecNeq(cross_norm1,none_cross_norm) && vecEq(cross_norm2,none_cross_norm))

            vec1 = point1 - proj1;
            vec2 = point2 - proj2;
            dist1 = dot(n,vec1);
            dist2 = dot(n,vec2);


            cross_pos = cross_point(segp1,segp2,proj1,proj2);



            dist_m = dist1 + (cross_pos(1) - proj1(1))/(proj2(1) - proj1(1))*(dist2 - dist1);

            if vecNeq(cross_norm2,none_cross_norm)
                dist_comp = dist2;
            else
                dist_comp = dist1;
            end

            if dist_m >= dist_comp 
                dist = dist_comp - radius;
            else
                dist = dist_m - radius;
            end
        end



        if (vecNeq(cross_norm1,none_cross_norm) && vecNeq(cross_norm2,none_cross_norm))

            vec1 = point1 - proj1;
            vec2 = point2 - proj2;
            dist1 = dot(n,vec1);
            dist2 = dot(n,vec2);
            if dist1 >= dist2 
                dist = dist2 - radius;
            else
                dist = dist1 - radius;
            end
        end
    end
    

    if dist < d
        d = dist;
    end
end
end