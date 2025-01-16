function [status,robotnew]=robottracking(t,robot)
status=0;


[J,H]=JacobiDiff(robot.DH, robot.nlink, robot.pos{t,robot.nlink}.p(:,2), robot.x(robot.nlink+1:robot.nlink*2,t));


U=robot.goal(7:9,t)-1*eye(3)*(robot.wx(1:3,t)-robot.goal(1:3,t))-1.5*eye(3)*(robot.wx(4:6,t)-robot.goal(4:6,t));
robot.u(:,t)=pinv(J)*[U-H*robot.x(robot.nlink+1:robot.nlink*2,t);0;0;0];



robot.x(1:2*robot.nlink,t+1)=robot.A*robot.x(:,t)+robot.B*robot.u(:,t);
for i=1:robot.nlink
    robot.x(i,t+1)=mod(robot.x(i,t+1)+pi,2*pi)-pi;
    robot.DH(i,1)=robot.x(i,t+1); 
end

[newpos,M]=CapPos(robot.base,robot.DH,robot.cap);
for i=1:robot.nlink
    robot.pos{t+1,i}=newpos{i};
end
robot.profile{t+1}.M=M;
robot.wx(1:3,t+1)=robot.pos{t+1,robot.nlink}.p(:,2);
[J,~]=JacobiDiff(robot.DH, robot.nlink, robot.pos{t+1,robot.nlink}.p(:,2), robot.x(robot.nlink+1:robot.nlink*2,t+1));

robot.wx(4:6,t+1)=J(1:3,:)*robot.x(robot.nlink+1:2*robot.nlink,t+1);

robotnew=robot;
end