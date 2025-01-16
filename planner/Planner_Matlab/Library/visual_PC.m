function [] = visual_PC(PC, PC_idx)
    if nargin < 2
        PC_idx = ones(size(PC, 1), 1);
    end
    for i=1:max(PC_idx)
        curPC = PC(PC_idx == i, :);
        scatter3(curPC(:, 1), curPC(:, 2), curPC(:, 3),0.5, "o",'MarkerFaceAlpha',1,'MarkerEdgeAlpha',1);
        hold on;
    end
    grid on;
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    set(gca,'ztick',[]);
    axis equal;
end

