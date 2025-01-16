function [PC, PC_idx] = load_PC(k, resolution)
    if nargin < 1
        k = 1;
    end
    if nargin < 2
        resolution = 0.00001;
    end
    
    [PC_x, PC_y, PC_z] = xyzread('data/sampled_wp.xyz');
    PC = [PC_x,PC_y,PC_z];
    PC_fixture = stlread('figure/fixture_PC_in.STL').Points;
    PC = [PC; PC_fixture];





    seed = 2;
    rng(seed);
    
    ptCloud = pointCloud(PC);
    downsampled = pcdownsample(ptCloud, 'gridAverage', resolution);
    PC_sampled = downsampled.Location;

    [~, idx_sampled_raw] = ismember(PC_sampled, PC, 'rows');
    idx_sampled = idx_sampled_raw(idx_sampled_raw > 0);

    PC_sampled = PC(idx_sampled, :);
    PC_idx = zeros(1, size(PC,1));
    
    idx_cluster = kmeans(PC_sampled, k,'MaxIter',300);
    
    for i=1:k
        curIdx = idx_sampled(idx_cluster == i);
        PC_idx(curIdx) = i;
    end

    hull = convhull(PC);
    PC_hull = PC(hull,:);
    PC_hull = unique(PC_hull, 'rows');
    [~, idx_hull_raw] = ismember(PC_hull, PC, 'rows');
    idx_hull = idx_hull_raw(idx_hull_raw > 0);
    PC_idx(idx_hull) = -1 * abs(PC_idx(idx_hull));


    

end

