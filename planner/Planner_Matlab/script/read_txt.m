clear


fid = fopen('weldTraj.txt','rt');
C = textscan(fid,'
fclose(fid);
data = [];
for i = 1:3
    tmp = cell2mat(C(i));
    data = [data tmp];
end
arr = data;
save('data/real_weld1.mat','arr')
















