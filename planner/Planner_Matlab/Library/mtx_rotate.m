function T_r = mtx_rotate(n, a)
    if nargin == 1
        a = zeros(1,3);
    end
    theta = norm(n);
    T_t_a = mtx_translate(a);
    k = n;
    if theta ~= 0
        k = n/theta;
    end
    k_x = k(1);
    k_y = k(2);
    k_z = k(3);
    K = [0, -k_z, k_y;
         k_z, 0, -k_x;
         -k_y, k_x, 0];
    R = eye(3) + sin(theta)*K + (1-cos(theta))*K*K;
    T_r = eye(4);
    T_r(1:3,1:3) = R;
    T_r = mtx_translate(a)*T_r*mtx_translate(-a);
end
