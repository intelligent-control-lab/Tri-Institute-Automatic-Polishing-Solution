function norm_vec = extract_norm_vector(PC, arr, ref_point, ref_normvec)
    PC_new = [arr';PC];
    arr_idx = [1:size(arr,2)];
    ptCloud = pointCloud(PC_new);

    k = 100;
    

    normals = pcnormals(ptCloud, k);


    figure;
    pcshow(ptCloud);
    title('Point Cloud with Normals');
    
    
    if nargin == 4
        normals(arr_idx,:) = ref_normvec;
    else
        for i=1:size(arr_idx,2)
            idx = arr_idx(i);
            v1 = ref_point - PC_new(idx,:);
            if dot(v1, normals(idx,:)) < 0
                normals(idx,:) = -1 * normals(idx,:);
            end
        end
    end    


    hold on;
    quiver3(ptCloud.Location(arr_idx,1), ptCloud.Location(arr_idx,2), ptCloud.Location(arr_idx,3), ...
            normals(arr_idx,1), normals(arr_idx,2), normals(arr_idx,3), 'r');
    hold off;
    
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    legend('Point Cloud', 'Normals');
    norm_vec = normals(arr_idx,:);
end

