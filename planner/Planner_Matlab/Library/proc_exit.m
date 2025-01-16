



enter_horizon = size(safe_theta,2) - 1;

goal_inv = [];

exit_start = robot.goal(:,enter_horizon) + [0;0.01;0];
for i = 1:5
    tmp = c_pre + (exit_start - c_pre) / 5 * i;
    goal_inv = [goal_inv,tmp];
end

for i = 1:tfinal
    tmp = robot.goal(:,enter_horizon-i+1) + [0;0.05;0];
    goal_inv = [goal_inv,tmp];
end

last = tmp;
for i = 1:nstep
    tmp = last + (robot.goal(:,1) - last) / nstep * i;
    goal_inv = [goal_inv,tmp];
end

exit_horizon = size(goal_inv,2);

t = 1;
while (t<= exit_horizon)
    disp(t);
    c_next = goal_inv(1:3,t);

    col_flag = 1;
    while(norm(c_next-c_pre)>thres || col_flag == 1)
        [theta_new, need_flag, eq_diff] = safetrack(theta_pre, robot, c_next,planes,LineSegs,anchor_point,consider_line);

        c_pre = ForKine(theta_new, robot.DH, robot.base, robot.cap);
        theta_pre = theta_new;
        col_flag = check_collision_complete(theta_pre,robot,planes,LineSegs,anchor_point,consider_line);
    end
    safe_traj = [safe_traj c_pre];
    safe_theta = [safe_theta theta_pre];

    t=t+1;
end