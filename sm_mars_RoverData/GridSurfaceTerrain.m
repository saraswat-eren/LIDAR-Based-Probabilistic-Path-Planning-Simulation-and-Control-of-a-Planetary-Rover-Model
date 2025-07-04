%% Split Data from a .mat File

% This script loads a .mat file containing a matrix of 3D points
% and splits that matrix into seperate x, y, and z variable

%% 2. Load the .mat File

% Make sure 'height_points_xyz.mat' is in the current MATLAB folder.
load("height_points_xyz.mat");
run("RoverPathPRM.m");
%% 3. Identify and Split the Data Matrix

% Now, split the matrix into x, y, and z columns, just like before.
x = points(:, 1);  % First column is X
y = points(:, 2);  % Second column is Y
z = points(:, 3);  % Third column is Z

% Create Grid Vectors & Z height parameters for grid surface block

xg = linspace(min(x), max(x), 256); % x-grid vector
yg = linspace(min(y), max(y), 144); % y-grid vector

% Create an interpolant that fits a surface of the form z = F(x,y)  
F = scatteredInterpolant(x,y,z);
z_heights = 0.5*F({xg,yg}); %  Using this syntax to conserve memory when querying a large grid of points.
roverPath_z = 0.5*F(roverPath_x, roverPath_y);
%defining sample position
z_sample=0.5*F(x_sample,y_sample);
run("Initial_orientation.m");
t0 = struct( 'pzOffset', roverPath_z(1)+0.5323, 'yaw', 0, 'pitch', 0, 'roll', 0);
roverPath = struct('x', roverPath_x, 'y', roverPath_y, 't0', t0, 'z', roverPath_z,  'Offset_vis_z', 0.1);% Create a structure with 6 fields
fileName = 'RoverPath.mat'; % Choose a filename
save(fileName, 'roverPath');
