



safe_theta_bar = safe_theta;
safe_theta_bar(2,:) = safe_theta(2,:) + pi/2;
safe_theta_bar(3:6,:) = -1*safe_theta(3:6,:);


start_exe = safe_theta_bar(:,1:nstep);
exe = safe_theta_bar(:,nstep:before_exit_length);
exe_exit = safe_theta_bar(:,before_exit_length+1:end);;
dlmwrite('results/safe_theta_start2exe.txt',start_exe');
dlmwrite('results/safe_theta_exec.txt',exe');
dlmwrite('results/safe_theta_exit.txt',exe_exit');
