safe_theta = safe_theta_old;
safe_theta(2,:) = safe_theta(2,:) + pi/2;
safe_theta(3:6,:) = -safe_theta(3:6,:);
safe_theta(6,:) = safe_theta(6,:) + pi/6;
filename = 'safe_theta.txt';

dlmwrite(filename, safe_theta')