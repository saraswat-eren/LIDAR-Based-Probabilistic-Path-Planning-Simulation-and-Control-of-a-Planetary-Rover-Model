
%% --- Step 1: Load Height Map Image ---
imgb = imread('mars_obstacle_local_height (21).png');
img=imresize(imgb,0.25);% Replace with your height map image
img_gray = double(img); % Convert to double for processing

% Normalize the height map (0 to 1)
img_norm = img_gray / 255;

% Define map size in km (if needed)
terrain_length_km = 30; % x-direction
terrain_width_km = 30;  % y-direction
height_max_km = 2;     % z-direction

% Define resolution
[rows, cols] = size(img_norm);
x = linspace(0, terrain_length_km, cols);
y = linspace(0, terrain_width_km, rows);
[X, Y] = meshgrid(x, y);
Z = img_norm * height_max_km;

%% --- Step 2: Generate Binary Occupancy Map ---
% Define slope/height threshold to detect obstacles
obstacle_threshold = 0.5; % Anything above this is an obstacle

obstacle_mask = img_norm > obstacle_threshold;

figure;
imshow(obstacle_mask);
title('Binary Occupancy Map');

% Create binaryOccupancyMap object (from Navigation Toolbox)
map = binaryOccupancyMap(obstacle_mask, terrain_length_km / cols);
figure;
show(map);
title('Occupancy Grid Map');

%% --- Step 3: Generate 3D Terrain Surface (Mesh) ---
% Create a 3D surface
figure;
surf(X, Y, Z, 'EdgeColor', 'none');
colormap(gray);
title('3D Terrain Surface');
xlabel('X [km]');
ylabel('Y [km]');
zlabel('Z [km]');
view(3);
axis tight;

%% --- Step 4: Create Mesh for STL ---
% Convert surface to mesh (vertices and faces)
fv = surf2patch(X, Y, Z, 'triangles');

% Optionally remove parts below a threshold (mask obstacle only)
fv.vertices(obstacle_mask(:), 3) = height_max_km; % flatten obstacles if needed

%% --- Step 5: Export to STL File ---
stlwrite('mars_obstacle_map.stl', fv);
disp('STL file exported as mars_obstacle_map.stl');