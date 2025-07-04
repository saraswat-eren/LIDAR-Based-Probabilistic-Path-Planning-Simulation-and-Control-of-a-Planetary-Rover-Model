% Mars Rover Simulation: LiDAR, DEM, Point Cloud, and Grid Map
% with A* Dynamic Path Planning and Obstacle Inflation
clear all; close all; clc;

% --- Configuration ---
% Define Global Map Dimensions
GLOBAL_MAP_HEIGHT = 1024;
GLOBAL_MAP_WIDTH  = 576;
% Waypoint file and variable configuration
WAYPOINT_MAT_FILENAME = 'roverPath_xy.mat'; % Ensure you have this file
WAYPOINTS_FULLMAP_VAR_NAME_IN_MATFILE = 'path';
SWAP_COLUMNS_AFTER_LOAD = false; % Set to true if your MAT file has [col, row]
% Padding around the waypoint bounding box to define the ROI (rows, cols)
WAYPOINT_PADDING = [25, 25];
% Simulation parameters
ROVER_MAX_RANGE = 30;
LIDAR_HEIGHT_DIFF_THRESHOLD = 0.1; 
DEM_SMOOTH_FACTOR = 1.0;
SLOPE_THRESHOLD = 0.30;
ELEVATION_THRESHOLD = 0.90;

% --- CONFIGURATION FOR ROVER SIZE ---
% Define a safety radius around the rover in grid cells.
% This will "inflate" obstacles to keep the path at a safe distance.
% You can tune this value. A larger value creates more cautious paths.
ROVER_RADIUS_CELLS = 2; 

% Terrain Change Parameters (Applied ONCE to ROI before simulation)
TERRAIN_CHANGE_CENTER_ROI_FRAC = [0.5, 0.5]; % Fractional position in ROI
TERRAIN_CHANGE_RADIUS_FRAC_OF_ROI_MIN_DIM = 0.1;
TERRAIN_CHANGE_MAX_HEIGHT = 0.2; % This will be the fixed height of the change

% --- Step 1: Load and preprocess the Mars surface image (Full Map) ---
try
    img_full = imread('mars surface.png'); % Ensure this filename is correct
    fprintf('Successfully loaded "mars_surface.png".\n');
catch
    fprintf('Warning: "mars_surface.png" not found. Using a placeholder gradient image of size %d x %d.\n', GLOBAL_MAP_HEIGHT, GLOBAL_MAP_WIDTH);
    [x_grad, y_grad] = meshgrid(1:GLOBAL_MAP_WIDTH, 1:GLOBAL_MAP_HEIGHT);
    img_full = uint8(mod(x_grad + y_grad, 256));
end
if size(img_full, 3) == 3
    img_full = rgb2gray(img_full);
end
img_full_resized = imresize(img_full, [GLOBAL_MAP_HEIGHT, GLOBAL_MAP_WIDTH]);
height_map_full = double(img_full_resized) / 255;
[full_map_height, full_map_width] = size(height_map_full);
fprintf('Full map initialized to %d rows x %d columns.\n', full_map_height, full_map_width);
figure('Name', 'Full Mars Surface Height Map');
imshow(img_full_resized);
title(sprintf('Full Mars Surface Height Map (%d x %d)', full_map_height, full_map_width));
hold on;

% --- Step 2: Load Waypoints (in Full Map Coordinates) ---
WAYPOINTS_FULLMAP = [];
try
    fprintf('Attempting to load FULL MAP waypoints from %s, variable "%s"...\n', WAYPOINT_MAT_FILENAME, WAYPOINTS_FULLMAP_VAR_NAME_IN_MATFILE);
    loaded_data_struct = load(WAYPOINT_MAT_FILENAME);
    if isfield(loaded_data_struct, WAYPOINTS_FULLMAP_VAR_NAME_IN_MATFILE)
        temp_waypoints_fullmap = loaded_data_struct.(WAYPOINTS_FULLMAP_VAR_NAME_IN_MATFILE);
        fprintf('Successfully found variable "%s" in %s.\n', WAYPOINTS_FULLMAP_VAR_NAME_IN_MATFILE, WAYPOINT_MAT_FILENAME);
        if SWAP_COLUMNS_AFTER_LOAD
            WAYPOINTS_FULLMAP = [temp_waypoints_fullmap(:,2), temp_waypoints_fullmap(:,1)];
        else
            WAYPOINTS_FULLMAP = temp_waypoints_fullmap;
        end
        if any(WAYPOINTS_FULLMAP(:,1) < 1) || any(WAYPOINTS_FULLMAP(:,1) > full_map_height) || any(WAYPOINTS_FULLMAP(:,2) < 1) || any(WAYPOINTS_FULLMAP(:,2) > full_map_width)
            error('Some waypoints are outside the full map boundaries.');
        end
        fprintf('Successfully loaded and validated full map waypoints.\n');
    else
        error('Variable "%s" not found in MAT-file "%s".', WAYPOINTS_FULLMAP_VAR_NAME_IN_MATFILE, WAYPOINT_MAT_FILENAME);
    end
catch ME
    disp(['ERROR during waypoint loading: ', ME.message]);
    disp('Using default waypoints due to error.');
    center_r = round(full_map_height/2); center_c = round(full_map_width/2);
    WAYPOINTS_FULLMAP = [center_r - 50, center_c - 50; center_r, center_c + 50; center_r + 50, center_c - 50];
end

% --- Step 3: Calculate Dynamic ROI based on Waypoints and Padding ---
min_wp_row = min(WAYPOINTS_FULLMAP(:,1)); max_wp_row = max(WAYPOINTS_FULLMAP(:,1));
min_wp_col = min(WAYPOINTS_FULLMAP(:,2)); max_wp_col = max(WAYPOINTS_FULLMAP(:,2));
roi_row_start = max(1, round(min_wp_row - WAYPOINT_PADDING(1)));
roi_row_end   = min(full_map_height, round(max_wp_row + WAYPOINT_PADDING(1)));
roi_col_start = max(1, round(min_wp_col - WAYPOINT_PADDING(2)));
roi_col_end   = min(full_map_width, round(max_wp_col + WAYPOINT_PADDING(2)));
roi_actual_width  = roi_col_end - roi_col_start + 1;
roi_actual_height = roi_row_end - roi_row_start + 1;
ROI_RECT_calculated = [roi_col_start, roi_row_start, roi_actual_width, roi_actual_height];
fprintf('Calculated ROI_RECT: [col_start=%d, row_start=%d, width=%d, height=%d].\n', roi_col_start, roi_row_start, roi_actual_width, roi_actual_height);
rectangle('Position', ROI_RECT_calculated, 'EdgeColor', 'g', 'LineWidth', 2, 'LineStyle', '--');
plot(WAYPOINTS_FULLMAP(:,2), WAYPOINTS_FULLMAP(:,1), 'y.-', 'MarkerSize',10);
legend({'Calculated ROI', 'Path (Full Map)'}, 'TextColor', 'black', 'Location', 'northeastoutside', 'Color', [0.9 0.9 0.9]); 
hold off;

% --- Step 4: Extract ROI and Transform Waypoints to ROI-Local Coordinates ---
height_map_roi_initial = height_map_full(roi_row_start:roi_row_end, roi_col_start:roi_col_end);
roi_dims = size(height_map_roi_initial);
PATH_WAYPOINTS_ROI = [WAYPOINTS_FULLMAP(:,1) - roi_row_start + 1, WAYPOINTS_FULLMAP(:,2) - roi_col_start + 1];

% --- Step 4.5: Generate FIXED Terrain Alteration (Mound) ---
actual_terrain_change_center_roi = [round(TERRAIN_CHANGE_CENTER_ROI_FRAC(1) * roi_dims(1)), round(TERRAIN_CHANGE_CENTER_ROI_FRAC(2) * roi_dims(2))];
actual_terrain_change_radius = round(TERRAIN_CHANGE_RADIUS_FRAC_OF_ROI_MIN_DIM * min(roi_dims));
[X_roi_mesh, Y_roi_mesh] = meshgrid(1:roi_dims(2), 1:roi_dims(1));
dist_from_center_fixed = sqrt((Y_roi_mesh - actual_terrain_change_center_roi(1)).^2 + (X_roi_mesh - actual_terrain_change_center_roi(2)).^2);
mound_effect_fixed = exp(-(dist_from_center_fixed.^2) / (2 * (actual_terrain_change_radius/2)^2));
height_change_fixed = TERRAIN_CHANGE_MAX_HEIGHT * mound_effect_fixed;
height_map_roi_fixed = height_map_roi_initial + height_change_fixed;
height_map_roi_fixed = min(max(height_map_roi_fixed, 0), 1);
fprintf('Fixed terrain alteration (mound) applied to ROI.\n');

% --- Step 5: Prepare for DEM and Updating Grid Map Visualization ---
dem_roi_static = imgaussfilt(height_map_roi_fixed, DEM_SMOOTH_FACTOR);
figure('Name', 'Static DEM of ROI');
surf(X_roi_mesh, Y_roi_mesh, dem_roi_static, 'EdgeColor', 'none');
colormap(jet); colorbar; title('Static Digital Elevation Model of ROI');
xlabel('X (cols)'); ylabel('Y (rows)'); zlabel('Elevation');
axis tight; view(30, 30);
% Setup for Updating Grid Map
figure_grid_map_updating = figure('Name', 'Updating ROI Grid Map');
ax_grid_map = axes('Parent', figure_grid_map_updating);
h_grid_map_img = imshow(zeros(roi_dims(1), roi_dims(2)), 'Parent', ax_grid_map, 'InitialMagnification', 'fit');
colormap(ax_grid_map, gray);
hold(ax_grid_map, 'on');
h_path_plot_on_grid = plot(ax_grid_map, NaN, NaN, 'c-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'm', 'DisplayName', 'Path Taken');
h_rover_pos_on_grid = plot(ax_grid_map, NaN, NaN, 'ys', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'DisplayName', 'Current Rover');
h_astar_path = plot(ax_grid_map, NaN, NaN, 'g-', 'LineWidth', 2, 'DisplayName', 'Planned A* Path');
title(ax_grid_map, 'Updating ROI Grid Map');
hold(ax_grid_map, 'off');

% --- Step 6: Simulate Rover Movement, LiDAR, and Dynamic Path Planning ---
master_waypoint_index = 1; 
current_rover_pos = PATH_WAYPOINTS_ROI(1, :);
dynamic_path = current_rover_pos; 
final_goal_pos = PATH_WAYPOINTS_ROI(end, :);
angles = linspace(0, 2*pi, 180);
cumulative_lidar_points_roi = [];
grid_map_roi_current = zeros(roi_dims(1), roi_dims(2));

fprintf('Starting simulation with dynamic path planning...\n');

while norm(current_rover_pos - final_goal_pos) > 2.0 && master_waypoint_index <= size(PATH_WAYPOINTS_ROI, 1)
    
    % --- 6.1: Define Goal & Perform LiDAR Scan ---
    current_goal_pos = PATH_WAYPOINTS_ROI(master_waypoint_index, :);
    rover_current_row = round(current_rover_pos(1));
    rover_current_col = round(current_rover_pos(2));
    if rover_current_row < 1 || rover_current_row > roi_dims(1) || rover_current_col < 1 || rover_current_col > roi_dims(2)
        warning('Rover is outside ROI bounds. Stopping simulation.'); break;
    end
    current_scan_lidar_points = [];
    rover_height_at_current_pos = height_map_roi_fixed(rover_current_row, rover_current_col);
    for i = 1:length(angles)
        theta = angles(i);
        for r = 1:ROVER_MAX_RANGE
            scan_pt_row_idx = round(rover_current_row + r * sin(theta));
            scan_pt_col_idx = round(rover_current_col + r * cos(theta));
            if scan_pt_row_idx > 0 && scan_pt_row_idx <= roi_dims(1) && scan_pt_col_idx > 0 && scan_pt_col_idx <= roi_dims(2)
                height_at_scan_point = height_map_roi_fixed(scan_pt_row_idx, scan_pt_col_idx);
                if (height_at_scan_point - rover_height_at_current_pos) > LIDAR_HEIGHT_DIFF_THRESHOLD
                    current_scan_lidar_points = [current_scan_lidar_points; scan_pt_row_idx, scan_pt_col_idx, height_at_scan_point];
                    break;
                end
            else; break; end
        end
    end
    cumulative_lidar_points_roi = unique([cumulative_lidar_points_roi; current_scan_lidar_points], 'rows');

    % --- 6.2: Update the Master Grid Map ---
    [dx, dy] = gradient(dem_roi_static);
    grid_map_roi_current(sqrt(dx.^2 + dy.^2) > SLOPE_THRESHOLD | dem_roi_static > ELEVATION_THRESHOLD) = 1;
    if ~isempty(cumulative_lidar_points_roi)
        linear_indices = sub2ind(roi_dims, cumulative_lidar_points_roi(:,1), cumulative_lidar_points_roi(:,2));
        grid_map_roi_current(linear_indices) = 1;
    end

    % --- 6.3: PLAN - Inflate Obstacles and Find Path with A* ---
    if ROVER_RADIUS_CELLS > 0
        se = strel('disk', ROVER_RADIUS_CELLS, 0);
        inflated_grid_map = imdilate(grid_map_roi_current, se);
    else
        inflated_grid_map = grid_map_roi_current;
    end
    fprintf('Planning path from [%.1f, %.1f] to waypoint %d [%.1f, %.1f]\n', ...
        current_rover_pos(1), current_rover_pos(2), master_waypoint_index, current_goal_pos(1), current_goal_pos(2));
    planned_path_segment = find_path_astar(inflated_grid_map, round(current_rover_pos), round(current_goal_pos));
    if isempty(planned_path_segment)
        fprintf('!!! PATH BLOCKED on inflated map. Cannot find a route. Stopping. !!!\n');
        break;
    end
    
    % --- 6.4: EXECUTE - Move Along the Planned Path ---
    if size(planned_path_segment, 1) > 1
        step_index = min(1 + 5, size(planned_path_segment, 1)); 
        next_step_pos = planned_path_segment(step_index, :);
    else
        fprintf('Rover is at or near the segment goal. No move needed.\n');
        next_step_pos = current_rover_pos;
    end
    current_rover_pos = next_step_pos;
    dynamic_path = [dynamic_path; current_rover_pos];
    
    % --- 6.5: Update Visualization and Check for Arrival ---
    if norm(current_rover_pos - current_goal_pos) < 2.0
        fprintf('>>> Arrived at major waypoint %d. <<<\n', master_waypoint_index);
        master_waypoint_index = master_waypoint_index + 1;
    end
    set(h_grid_map_img, 'CData', grid_map_roi_current);
    title(ax_grid_map, sprintf('ROI Grid Map - Heading to Waypoint %d/%d', master_waypoint_index, size(PATH_WAYPOINTS_ROI, 1)));
    set(h_astar_path, 'XData', planned_path_segment(:,2), 'YData', planned_path_segment(:,1));
    set(h_path_plot_on_grid, 'XData', dynamic_path(:,2), 'YData', dynamic_path(:,1));
    set(h_rover_pos_on_grid, 'XData', current_rover_pos(2), 'YData', current_rover_pos(1));
    legend(ax_grid_map, [h_path_plot_on_grid, h_rover_pos_on_grid, h_astar_path], 'Location', 'northeastoutside');
    drawnow;
    pause(0.1);
end
fprintf('Finished simulation loop.\n');

% --- Step 7: Final Visualizations ---
% Final Accumulated LiDAR Point Cloud
figure('Name', 'Final Accumulated LiDAR Point Cloud');
if ~isempty(cumulative_lidar_points_roi)
    scatter3(cumulative_lidar_points_roi(:, 2), cumulative_lidar_points_roi(:, 1), cumulative_lidar_points_roi(:, 3), 10, cumulative_lidar_points_roi(:, 3), 'filled');
    colormap(jet); colorbar; xlabel('X (cols)'); ylabel('Y (rows)'); zlabel('Elevation');
    title('Final Accumulated LiDAR Point Cloud'); grid on; axis tight; view(30, 30);
end
% Final Grid Map with Dynamic Path
figure('Name', 'Final ROI Grid Map with Dynamic Path');
imshow(grid_map_roi_current, []); colormap(gray); hold on;
plot(dynamic_path(:,2), dynamic_path(:,1), 'c-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'c', 'DisplayName', 'Path Taken');
plot(PATH_WAYPOINTS_ROI(:,2), PATH_WAYPOINTS_ROI(:,1), 'm--', 'LineWidth', 1, 'DisplayName', 'Original Waypoints');
plot(PATH_WAYPOINTS_ROI(1,2), PATH_WAYPOINTS_ROI(1,1), 'g*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
plot(PATH_WAYPOINTS_ROI(end,2), PATH_WAYPOINTS_ROI(end,1), 'r*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'End');
title('Final Grid Map with Dynamically Redefined Path');
legend('show', 'Location', 'best');
hold off;
disp('Simulation finished.');