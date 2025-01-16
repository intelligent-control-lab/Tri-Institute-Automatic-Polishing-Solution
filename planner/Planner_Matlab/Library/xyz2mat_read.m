
clc
clear
[x,y,z] = xyzread('data/welTraj.xyz')
arr = [x,y,z];
save('data/traj_new.mat','arr');