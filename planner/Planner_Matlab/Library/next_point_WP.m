function c_next = next_point_WP(wp_pos, c1, PC_origin)
    [curPC, M_PC] = processPC(PC_origin, wp_pos);
    c_next = setVertice(c1', M_PC)';
end
