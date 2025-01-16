dfunc = @(x) dist_arm_planes(x, robot, planes, LineSegs, anchor_point, consider_line);
ref_grad = num_grad_jac(dfunc,min);

posigrad = zeros(6,1);
negagrad = zeros(6,1);
for i = 1:6
    if ref_grad(i) >= 0
        posigrad(i) = ref_grad(i);
    else
        negagrad(i) = ref_grad(i);
    end
end

min = min - eta*posigrad;