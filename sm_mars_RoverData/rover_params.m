%% Script to setup parameters for the rover  

% Copyright 2021-2022 The MathWorks, Inc

%% Rover Assembly Params

Chasis.Top_Frame_R =...
    [   1.0  0  0
    0   0   1.0
    0  -1.0   0 ];

Chasis.Top_Frame_T = [-1.408435770788863e-09 0.360100000000000 -0.099999999882285];


Chasis.R_Frame_R =...
    [0   0   1.0
    1.0   0   0
    0   1.0   0];

Chasis.R_Frame_T = [0.500100000000000 0.170000002063840 0.100000002063841];

Chasis.L_Frame_R = ...
    [0.0  0  -1.0
    1.0   0.0  0
    0  -1.0   0.0];

Chasis.L_Frame_T = [-0.500100000000000 0.169999999356227 0.100000001576468];

Chasis.F4_Frame_R =...
    [1.0  0  0
    0   0   1.0
    0  -1.0   0];

Chasis.F4_Frame_T = [0 0 0];

DiffBar.ToChasis_R = ...
    [1.0 0   0
    0   0  -1.0
    0   1.0   0];
DiffBar.ToChasis_T = [0   0.0401 0] ;

DiffBar.F3_R = ...
    [1.0                   0                   0
    0   0  -1.0
    0   1.0   0];
DiffBar.F3_T = [-0.550000005278643  0   0.000000002311580] ;

DiffBar.F2_R = ...
    [-1.0                   0                   0
    0   0  -1.0
    0  -1.0   0];
DiffBar.F2_T = [0.549999994090071                   0   0.000000007967969] ;


Rtg.R = ...
    [1.0000         0         0
    0   -0.8660    0.5000
    0   -0.5000   -0.8660 ];

Rtg.T = [0.0000    0.3479   -1.0008];


Rtg_Bridge.T1 = [ -0.4400    0.4987   -0.7821];
Rtg_Bridge.T2 = [0.4400    0.4987   -0.7821];

ArmBase.R = ...
    [1.0000         0         0
    0    0.0000    1.0000
    0   -1.0000    0.0000];

ArmBase.T = [0.3800    0.2500    0.9800];

HomePose.R = ...
    [1.0000         0         0
    0    0.0000    1.0000
    0   -1.0000    0.0000];
HomePose.T = [0.3842    0.2251    0.8275];

Target.R = ...
    [1     0     0
    0     1     0
    0     0     1];
Target.T = [-0.0495    0.3775    0.8260];

CameraBase.T = [-0.400 0.3200 0.6500];

% MobilityPrimaryArmR
MPL.SRJ_R = ...
    [1.0000         0         0
    0    0.0000   -1.0000
    0    1.0000    0.0000
    ];
MPL.SRJ_T = [-0.5100   -0.0901    0.0600];

MPL.DR_R =...
    [-1.0000         0         0
    0    0.0000    1.0000
    0    1.0000    0.0000];
MPL.DR_T = [0.0000    0.0600   -0.2000];

MPL.LFB_R = ...
    [ 1.0000         0         0
    0   -1.0000         0
    0         0   -1.0000];
MPL.LFB_T = [0.9950   -0.1950    0.1520];

MPL.CRJ_R = ...
    [1.0000         0         0
    0    0.0000   -1.0000
    0    1.0000    0.0000
    ];
MPL.CRJ_T = [ -0.0000    0.1000   -0.0000];
%L
MPR.SRJ_R = ...
    [1.0000         0         0
    0    0.0000   -1.0000
    0    1.0000    0.0000
    ];
MPR.SRJ_T = [-0.5100   -0.0901   -0.0600];

MPR.DL_R =...
    [1.0000         0         0
    0    0.0000    1.0000
    0    -1.0000    0.0000];
MPR.DL_T = [0.0000    0.0625    0.2000];

MPR.RFB_R = ...
    [ 1.0000         0         0
    0   1.0000         0
    0         0   1.0000];
MPR.RFB_T = [0.9950   -0.1950    -0.1520];

MPR.CRJ_R = ...
    [1.0000         0         0
    0    0.0000   -1.0000
    0    1.0000    0.0000
    ];
MPR.CRJ_T = [ -0.0000    0.1000   -0.0000];
% MSL

MSL.PSJ_R = ...
    [1.0000         0         0
         0    0.0000    1.0000
         0   -1.0000    0.0000];
MSL.PSJ_T = [ 0 0 0];

MSL.SLB_R =...
    [1.0000         0         0
         0    0.0000    1.0000
         0    1.0000    0.0000];
MSL.SLB_T =[-0.4900    0.1100   -0.0950];

MSL.SLM_R = ...
    [1.0000         0         0
         0    0.0000    1.0000
         0   -1.0000    0.0000];
MSL.SLM_T = [0.5122    0.2901   -0.4162];

% MSR

MSR.PSJ_R = ...
    [1.0000         0         0
         0    0.0000    1.0000
         0   -1.0000    0.0000];
MSR.PSJ_T = [ 0 0 0];

MSR.SRB_R =...
    [ 1.0000         0         0
         0   -1.0000         0
         0         0   -1.0000];

MSR.SRB_T =[-0.4900    0.1100   0.0950];

MSR.SRM_R = ...
    [1.0000         0         0
         0    0.0000    1.0000
         0   -1.0000    0.0000];
MSR.SRM_T = [0.5122    0.2901   0.4162];

% SteeringBracket

SB.SRJ_R =...
    [0     0     1
     1     0     0
     0     1     0];
SB.SRJ_T = [0.3201   -0.0200   -0.0000];

SB.WRJ_R =...
    [1     0     0
     0     0     1
     0     -1     0];
SB.WRJ_T = [0.0000    0.0601    0.0000];

% Wheel
Wheel.WRJ_R =...
    [1     0     0
     0     0     1
     0     -1     0];
Wheel.WRJ_T = [0.0000   -0.0201    0.0000];

%
Chassis_L = 2;
Chassis_width = 1.761;
Wheel_Radius = 0.215; %m
rover_accel = 0.5; %m/s^2
t1 = 1; % roverStartTime

%% Wheel Point Cloud

points = stlread('mobility_wheel_outer.stl') ;
all_pts = unique(points.Points,'Rows');

sorted_rows = sortrows(all_pts,2);

outer_rim_pts = sorted_rows((sorted_rows(:,2)== min(sorted_rows(:,2))),:);
r = [sqrt(outer_rim_pts(:,1).^2+ outer_rim_pts(:,3).^2)];
outer_rim_pts = outer_rim_pts(r> 20.801,:);

middle_rim_pts_1 = sorted_rows((sorted_rows(:,2)== median(sorted_rows(:,2))),:);

inner_rim_pts = sorted_rows((sorted_rows(:,2)== max(sorted_rows(:,2))),:);
r = [sqrt(inner_rim_pts(:,1).^2+ inner_rim_pts(:,3).^2)];
inner_rim_pts = inner_rim_pts(r> 20.801,:);

new_wheel_points = [outer_rim_pts ; middle_rim_pts_1; inner_rim_pts];


%% PID Control Parameters for Wheel Speed Control and Steering Control

Kp = 1e4;
Ki = 50 ;
Kd = 0;

Kp_steer = 100;
Ki_steer = 1;
Kd_steer = 1;

%% Spatial Contact Force Block

K = 200000; 
D_contact = 1e4;
TW = 0.3;
Mu = 0.8;
Mud = 0.6;
Cv = 1e-2;

%% Other Params

steering_damping = 10;
wheel_damping = 1;
TrqUpLimit = 500;
TrqLoLimit = -500;