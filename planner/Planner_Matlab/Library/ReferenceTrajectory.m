





r=0.25;v=25;
posini = ForKine(theta_init, robot.DH, robot.base, robot.Msix2tool);


posexe = arr(:,1);
tfinal= size(arr, 2);
nstep = 15;
nwait = 1;



diff = posexe - posini;
robot.goal = [];
for t = 1:nstep
    robot.goal(1:3,t) = posini + t/nstep*diff;
end












for t = 1:nwait
    robot.goal(1:3,t+nstep) = posexe;
end




for t = 1:tfinal-1
    diff = arr(:,t+1) - arr(:,t);
    ministep = 1;
    for x = 1:ministep
        robot.goal = [robot.goal arr(:,t) + x / ministep * diff];
    end
end

tfinal = (tfinal - 1) * ministep;