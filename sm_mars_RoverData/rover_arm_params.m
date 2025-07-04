%% Script to setup parameters for the rover arm 

% Copyright 2021-2022 The MathWorks, Inc

%% Arm Joint Params
D = 0.5;
D2 = 0.5;

%% Arm PID Gains
Kp_arm = 200;
Ki_arm = 50;
Kd_arm = 5;

%% Arm Assembly Params
%============= RigidTransforms =============%
HomePos.R = ...
    [1.0000         0         0
         0    1.0000         0
         0         0    1.0000];
HomePos.T = [0.0042    0.1525   -0.0249];

R_disc_Link1 = 5; %cm

Link1.translation = [-15000 12000 3250];  % mm
Link1.angle = 0;  % rad
Link1.axis = [0 0 0];

Link2(1).translation = [-83300 -7627 -3250];  % mm
Link2(1).angle = 0;  % rad
Link2(1).axis = [0 0 0];


Link2(2).translation = [-15000 -7627 750];  % mm
Link2(2).angle = pi;  % rad
Link2(2).axis = [1 0 0];


Link3(1).translation = [-18700 -7627 8750];  % mm
Link3(1).angle = 0;  % rad
Link3(1).axis = [0.091717735705432957 -0.99578504555806036 -5.8733215973969995e-18];

Link3(2).translation = [-83300 -7627 4750];  % mm
Link3(2).angle = 0;  % rad
Link3(2).axis = [0 0 0];
