
function [col_flag, minDist] = check_collision_complete_PC_cluster(theta0,robot,PC, PC_idx)
col_flag = 0;
nPC = max(abs(PC_idx));
minDist = inf;
for j = 1:nPC
    curPC = PC(PC_idx == j, :);
    dist = dist_arm_PC(theta0, robot.DH, robot.base, robot.cap, curPC, robot.Msix2tool);
    minDist = min(dist, minDist);
    if minDist <= 0
        col_flag = 1;
        break
    end
end
