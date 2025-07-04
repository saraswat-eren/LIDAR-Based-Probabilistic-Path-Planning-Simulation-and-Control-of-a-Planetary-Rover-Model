%% MASTER SETUP SCRIPT FOR ROVER SIMULATION (FINAL, WORKING VERSION V2)
% This script configures all data and parameters for the Simulink model.
% It includes the modern, robust syntax for creating Bus Objects.

disp('--- Setting up simulation environment ---');

% --- Step 1: Load Core Data ---
disp('Loading height map data from height_points_xyz.mat...');
try
    loaded_data = load(['sm_mars_RoverData',filesep, 'height_points_xyz.mat']);
    variable_name = 'points';
    if ~isfield(loaded_data, variable_name)
        error('The variable "%s" was not found in the .mat file. Please check the name.', variable_name);
    end
    xyz_data = loaded_data.(variable_name);
    disp('Height map point cloud loaded successfully.');
catch
    error('Failed to load height_points_xyz.mat. Please ensure it is in the project folder.');
end

% --- Step 2: Convert Point Cloud to a Grid Matrix ---
disp('Converting point cloud to a grid matrix...');
X_points = xyz_data(:, 1);
Y_points = xyz_data(:, 2);
Z_points = xyz_data(:, 3);
grid_resolution = 1.0;
x_vector = min(X_points):grid_resolution:max(X_points);
y_vector = min(Y_points):grid_resolution:max(Y_points);
[X, Y] = meshgrid(x_vector, y_vector);
Z = griddata(X_points, Y_points, Z_points, X, Y);
Z = fillmissing(Z, 'linear');
disp('Grid matrix (Z) created successfully.');

% --- Step 3: Pre-compute the 3D Terrain Mesh ---
disp('Creating 3D terrain mesh for LiDAR simulation...');
terrainMesh.X = X;
terrainMesh.Y = Y;
terrainMesh.Z = Z;
terrainMesh.Triangulation = delaunay(X, Y);
terrainMesh.Vertices = [X(:), Y(:), Z(:)];
disp('Terrain mesh complete.');

% --- Step 4: Define All Simulation Parameters ---
disp('Defining simulation parameters...');
simParams.sensor.mountHeight = 0.5;
simParams.sensor.maxRange = 40.0;
simParams.sensor.minRange = 0.5;
simParams.sensor.h_fov = [-180, 180];
simParams.sensor.h_res = 0.5;
simParams.sensor.v_channels = 16;
simParams.sensor.v_fov = [-15, 15];
simParams.map.size = 25;
simParams.map.resolution = 1;
simParams.planner.lookahead = 7.0;
disp('All parameters defined.');

%% --- Step 5: Create Bus Objects for Simulink (Modern Syntax) ---
% This section creates the "blueprints" for our structs so Simulink understands them.
disp('Creating Simulink Bus Objects...');

% --- Bus for sensor parameters ---
elems_sensor(1) = Simulink.BusElement;