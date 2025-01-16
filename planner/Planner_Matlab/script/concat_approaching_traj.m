
clear
load("results/measure.mat");
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
theta_ini = [0; -52.6664; -32.3777; 0; -20.2886; 30.0001];

theta_ini = theta_ini - [0;0;0;0;0;30.0001];
theta_ini = theta_ini / 180 * pi;
theta_ini(3:6) = -1*theta_ini(3:6);
theta_ini(2) = theta_ini(2) - pi/2;

theta_medi = [0.8192; -36.3536; -31.5474; 0.8593; -24.1294; 108.4009];

theta_medi = theta_medi - [0;0;0;0;0;30.0001];
theta_medi = theta_medi / 180 * pi;
theta_medi(3:6) = -1*theta_medi(3:6);
theta_medi(2) = theta_medi(2) - pi/2;
theta_final = safe_theta(:,1);
step1 = 80;
step2 = 80;
diff1 = theta_medi - theta_ini;
diff2 = theta_final - theta_medi;
pre_theta = theta_ini;
for i = 1:step1
    tmp = theta_ini + diff1/step1*i;
    pre_theta = [pre_theta tmp];
end
for i = 1:step2 - 1
    tmp = theta_medi + diff2/step2*i;
    pre_theta = [pre_theta tmp];
end

safe_theta = [pre_theta safe_theta];
save('results/meaure_aug.mat','safe_theta')