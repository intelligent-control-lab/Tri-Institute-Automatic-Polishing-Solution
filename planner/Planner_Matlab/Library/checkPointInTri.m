function flag = checkPointInTri(pointP,pointT1,pointT2, pointT3)



cross_norm1 = hatw(pointT2 - pointT1)*(pointP - pointT1);
cross_norm2 = hatw(pointT3 - pointT2)*(pointP - pointT2);
cross_norm3 = hatw(pointT1 - pointT3)*(pointP - pointT3);

flag = 0;

lambda1 = dot(cross_norm2, cross_norm1);
lambda2 = dot(cross_norm3, cross_norm1);

if lambda1 > 0 && lambda2 > 0
   flag = 1;
end

end
