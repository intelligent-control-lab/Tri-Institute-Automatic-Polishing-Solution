function T_t = mtx_translate(t)

    T_t = eye(4);
    for i=1:1:3
        T_t(i,4) = t(i);
    end
end