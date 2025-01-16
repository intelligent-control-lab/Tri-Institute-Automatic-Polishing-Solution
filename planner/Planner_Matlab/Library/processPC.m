function [PC, M_PC, base_point, center_point] = processPC(PC, PC_pos)
    M1 = [0.91647976 -0.00729021  0.40001462  1.40608102;
          0.02281488  0.99915928 -0.03406199 -0.02509452;
         -0.39943     0.0403434   0.91587558  1.04437673;
          0.          0.          0.          1.   ];

    x = PC(:,1);
    idx = find(x==0);
    PC_x0 = PC(idx,:);
    center_point = mean(PC_x0);
    mean_point = mean(PC)';
    base_point = mean_point';
    base_point(3) = min(PC(:, 3)) - 0.3;
    positioner_offset = [2, 0, 0];
    positioner2basepoint = [-0.10089, 0, 0.380];
    positioner2basepoint(3) = positioner2basepoint(3) + 0.005;
    base2workpiece = [-0.53376, 0, 0.2782];
    base_point = positioner_offset + positioner2basepoint;
    workpiece_offset = base_point + base2workpiece;
    M_PC = [1. 0. 0. workpiece_offset(1);
          0. 1. 0. workpiece_offset(2);
          0. 0. 1. workpiece_offset(3);
          0. 0. 0. 1.];
    if nargin >= 2
        for i=1:3
            v = zeros(1,3);
            v(i) = 1;
            if PC_pos(i) == 0
                continue;
            end
            M_PC = mtx_rotate(v*PC_pos(i), base_point) * M_PC;
        end
    end
      M_PC = [0.9408    0.0255    0.3380    1.4782
               -0.0237    0.9997   -0.0096   -0.0050
               -0.3381    0.0010    0.9411    0.8205
                     0         0         0    1.0000];
    PC = setVertice(PC,M_PC);
    center_point = setVertice(center_point,M_PC)';
end

