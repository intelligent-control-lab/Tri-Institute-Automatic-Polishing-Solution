function eq = vecEq(vec1, vec2)

    diff = vec1 - vec2;
    if (norm(diff) < 0.1)
        eq = 1;
    else
        eq = 0;
    end
end