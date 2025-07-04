%% Script to setup parameters for the sm_mars_rover model 

% Copyright 2021-2022 The MathWorks, Inc

%% Mars Rover Params

run(['sm_mars_RoverData',filesep,'rover_params']) ; 

%% Rigid Terrain Params

run(['sm_mars_RoverData',filesep,'GridSurfaceTerrain']);

%% Sample Position

sample_position = [x_sample  y_sample z_sample];

%% Rover Path Planning and Control Params
goal_loc = [roverPath.x(end) roverPath.y(end)];
% Points for visualizing the waypoints
pointCloud=[roverPath_x, roverPath_y, roverPath_z];

%% Rover Arm Params
run(['sm_mars_RoverData',filesep,'rover_arm_params']);
arm_q0 =  [-2.3554 -125.9144   49.8193 -175.7336  177.6446  180.0000];
%% Rover Arm Planning And Control
load(['sm_mars_RoverData',filesep,'roverArmTaskSpaceConfig.mat']);

%% Rover Camera Assmebly Params
run(['sm_mars_RoverData',filesep,'rover_camera_params']) ;
