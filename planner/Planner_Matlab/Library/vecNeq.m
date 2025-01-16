function neq = vecNeq(vec1, vec2)

    diff = vec1 - vec2;
    if (norm(diff) < 0.1)
        neq = 0;
    else
        neq = 1;
    end
end