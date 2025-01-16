
clear
[x y z] = xyzread('data/weld_traj.xyz')
arr = [x y z];
save('data/real_weld.mat','arr');