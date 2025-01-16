fid = fopen('safe_theta.txt','rt');
C = textscan(fid,'
fclose(fid);
data = [];
for i = 1:6
    tmp = cell2mat(C(i));
    data = [data tmp];
end
safe_theta = data;
safe_theta = safe_theta';
safe_theta(6,:) = safe_theta(6,:) - pi/6;
safe_theta(3:6,:) = -safe_theta(3:6,:);
safe_theta(2,:) = safe_theta(2,:) - pi/2;
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
load('data/real_weld.mat');
arr = arr(1:60,:);
arr = arr';
visualize_traj_realsetup(safe_theta,arr);
