





r=0.25;v=25;
posini = ForKine(theta_init, robot.DH, robot.base, robot.Msix2tool);


posexe = arr(:,1);
tfinal= size(arr, 2);
nstep1 = 10;
nstep2 = 10;
nstep3 = 20;
nwait = 0;


robot.goal = [];

diff1 = center_point - posini;
diff2 = posexe - center_point;
diff3 = posexe - posini;
if track_center == 1
    for t = 1:nstep1
        robot.goal(1:3,t) = posini + t/nstep1*diff1;
    end
    for t = 1:nstep2
        robot.goal(1:3,t + nstep1) = center_point + t/nstep2*diff2;
    end
else
    for t = 1:nstep3
        robot.goal(1:3,t) = posini + t/nstep3*diff3;
    end
end

nstep = nstep1 + nstep2;

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