



safe_theta_bar = safe_theta;
safe_theta_bar(2,:) = safe_theta(2,:) + pi/2;
safe_theta_bar(3:6,:) = -1*safe_theta(3:6,:);




home  = [0, -0.38, -0.42, 0, -0.38, 0.42]';
measure_init = safe_theta_bar(:,1);
nstep_start = 10;
start2exe = [home];
for i = 1:nstep_start
    tmp = home + (measure_init - home)/nstep_start*i;
    start2exe = [start2exe tmp];
end
safe_theta_bar = [start2exe safe_theta_bar];

len = size(safe_theta_bar,2);
out = [];
for i = 1:len
    out = [out safe_theta_bar(:,len-i+1)];
end
total = [safe_theta_bar out];
dlmwrite('results/safe_theta_measure.txt',total');