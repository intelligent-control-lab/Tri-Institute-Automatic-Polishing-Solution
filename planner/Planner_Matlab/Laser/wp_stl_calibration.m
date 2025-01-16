clc
clear
close all;


file_idx = [1,2];
PC_measure = [];
PC_CAD = stlread('WP_outerface.STL').Points;
for i=1:size(file_idx,2)
    file_name_theta_measurement = join(['data/wp_PC/in0227/', num2str(file_idx(i)), '.txt'],"");
    PC_cur = load(file_name_theta_measurement); 
    PC_measure = [PC_measure;PC_cur];
end

seed = 2;
rng(seed);


ptCld_CAD = pointCloud(PC_CAD);
ptCld_measure = pointCloud(PC_measure);
ptCld_measure = pcdenoise(ptCld_measure,Threshold=0.001);



ptCloud_CAD = pcdownsample(ptCld_CAD,gridAverage=0.01);
ptCloud_measure = pcdownsample(ptCld_measure,gridAverage=0.01);



trans = [1.0 0 0];
tform = rigidtform3d([0 0 0],trans);
ptCloud_measure_Tformed = ptCloud_measure;

ptCloud_CAD = pctransform(ptCloud_CAD,tform);

figure;
pcshowpair(ptCloud_CAD, ptCloud_measure_Tformed)
axis on
xlim([-1 3])
ylim([-1 2])
legend("Original","Transformed",TextColor=[1 1 0]);

[estimatedTform, ~ , rmse] = pcregistericp(ptCloud_measure_Tformed, ptCloud_CAD, Metric="planeToPlane",MaxIterations=8000, Tolerance=[0.001,0.001]);


ptCloud_new = pctransform(ptCloud_CAD,invert(estimatedTform));
disp(invert(estimatedTform).A)
disp(tform.A)
rmse
inv(estimatedTform.A) * tform.A
figure;
pcshowpair(ptCloud_new, ptCloud_measure_Tformed);
axis on
xlim([1 3])
ylim([-1 2])
title("Aligned Point Clouds")