res = load('./results/test/res_cluster.txt');
total_time = res(:,1)
solution_len = res(:,2)
fail_num = res(:,3)
mean_iteration = res(:,4)
PC_k = res(:,5)
PC_num = res(:,6)
PC_resolution = res(:,7)
x = 1:1:size(res,1)
x = PC_k;
figure(1)
linewidth = 1;
yyaxis left;
plot(x, total_time./solution_len, '*-', 'LineWidth', linewidth, 'DisplayName', 'mean time');
ylabel('time/s');
hold on;
yyaxis right;
plot(x, solution_len, '*-', 'LineWidth', linewidth, 'DisplayName', 'solution len');
ylabel('# of points');
xlabel('resolution/m');
