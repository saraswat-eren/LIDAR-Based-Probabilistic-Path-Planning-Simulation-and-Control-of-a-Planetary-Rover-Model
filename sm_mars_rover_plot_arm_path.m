%% Script to plot the end effector path

% Copyright 2021-2022 The MathWorks, Inc.
if exist('sm_mars_rover_out') == 0
sm_mars_rover_out = sim("smMarsRover");
end
load(['sm_mars_RoverData',filesep,'roverArmTaskSpaceConfig.mat']);

sampleLocation = sm_mars_rover_out.logsout{2}.Values.Data(end,:);
sampleLocation_Approach = [sampleLocation(1) sampleLocation(2) sampleLocation(3)+0.1];
wayPoints_Task= [eeConfig.Targets(1,1:3);...
                 eeConfig.Targets(2,1:3);...
                 sampleLocation_Approach;
                 sampleLocation;
                 eeConfig.Targets(3,1:3);...
                 eeConfig.Targets(4,1:3)];

ee = sm_mars_rover_out.logsout{1}.Values.Data;

figure;
hold on ;

plot3(wayPoints_Task(:,1),wayPoints_Task(:,2),wayPoints_Task(:,3),'ko','LineWidth',0.5,'DisplayName','Waypoints')
axis auto

title('End Effector Trajectory');

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
view(-82.2000,9.6)
plot3(ee(:,1),ee(:,2),ee(:,3),'g--','LineWidth',2,'DisplayName','Trajectory');
plot3(wayPoints_Task(1,1), wayPoints_Task(1,2), wayPoints_Task(1,3),'bd','LineWidth',2,'DisplayName','Home Position');
plot3(sampleLocation(1), sampleLocation(2), sampleLocation(3),'md','LineWidth',2,'DisplayName','Grab Sample');
plot3(wayPoints_Task(6,1), wayPoints_Task(6,2), wayPoints_Task(6,3),'rd','LineWidth',2,'DisplayName','Store Sample');

legend('Location','Best');
axis equal
grid on
box on
movegui('center');
hold off;