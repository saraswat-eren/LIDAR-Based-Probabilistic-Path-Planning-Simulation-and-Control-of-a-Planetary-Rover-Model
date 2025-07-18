% Replace with the actual filename

% Assumes the variable is named 'map' and is of class binaryOccupancyMap

% Show the

imgb= imread('mars_obstacle_local_height (21).png');

img=imresize(imgb,0.25);
grayImg = im2gray(img); % Convert to grayscale

bwImg = imbinarize(grayImg); % Convert to binary map (0s and 1s)

map = binaryOccupancyMap(~bwImg, 1); % Flip to make white as free space

figure;

show(map);

title('Binary Occupancy Map');

% Robot definition

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% Inflate map to account for robot size

mapInflated = copy(map);

inflate(mapInflated, robot.TrackWidth/2);

% Create PRM for path planning

prm = robotics.PRM(mapInflated);

prm.NumNodes =5000;

prm.ConnectionDistance = 20;

% Define start and goal locations

startLocation = [50 70]; % Adjust based on your map

endLocation = [225 75]; % Adjust based on your map

% Find the path

path = findpath(prm, startLocation, endLocation);

% Safety check
if isempty(path)
    disp('No path found. Try adjusting NumNodes or ConnectionDistance.');
    return; % Exit before trying to index an empty path
end

row=size(path,1);
% Show PRM and path
show(prm);

% Step 2: Optimize the raw path
options=optimizePathOptions;
options.MinTurningRadius = 2;
options.MaxAngularVelocity=0.01;
options.MaxSolverIteration=5;
optPath = optimizePath(path, mapInflated,options);  % assuming optPath is user-defined
globalPath_x = path(:, 1);  % First column is X
globalPath_y = path(:, 2);  % Second column is Y
show(map);
hold on;
plot(globalPath_x, globalPath_y, 'r-', 'LineWidth', 2);
title('Path Found via PRM');
disp('Map created')

% Compute cumulative distance (arc length)
distances = [0; cumsum(sqrt(diff(globalPath_x).^2 + diff(globalPath_y).^2))];  % Parametrize by real distance

% Define how many smoothed points to generate
numPoints = 300;
tq = linspace(0, distances(end), numPoints);  % Even spacing over arc length

% Perform spline interpolation based on arc length
roverPath_x = spline(distances, globalPath_x, tq);
roverPath_y = spline(distances, globalPath_y, tq);
roverPath_x=roverPath_x';
roverPath_y=roverPath_y';
run("SamplePosition.m");
% Plot smoothed path
figure;
show(map); hold on;
plot(roverPath_x, roverPath_y, 'g-', 'LineWidth', 2);           % Smoothed path
plot(globalPath_x, globalPath_y, 'ro--', 'MarkerSize', 3);
plot(x_sample, y_sample, 'b*', 'MarkerSize',10);  % Dots with specific size
legend('Smoothed Path','Original PRM Waypoints','sample position');
title('Smoothed PRM Path with Arc-Length-Based Spline');
