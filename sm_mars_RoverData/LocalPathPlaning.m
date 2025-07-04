
scaleX = 2048 / 256;  % = 8
scaleY = 1152 / 144;  % = 8
hiRes_x = roverPath_x * scaleX;
hiRes_y = roverPath_y * scaleY;
% Inputs:
MartianSurface=imread('Martian_surface.png');
x1=roverPath.x(1);y1=roverPath.y(1);
x2=roverPath_x(2);y2=roverPath.y(2);
% x1, y1 and x2, y2: consecutive waypoints (already scaled to hi-res)

% Define margin (in pixels)
margin = 100; % you can adjust this based on expected Lidar range

% Bounding box corners
xMin = max(min(x1, x2) - margin, 1);
xMax = min(max(x1, x2) + margin, size(MartianSurface, 2));
yMin = max(min(y1, y2) - margin, 1);
yMax = min(max(y1, y2) + margin, size(MartianSurface, 1));

% Crop the image region
localPatch = MartianSurface(round(yMin):round(yMax), round(xMin):round(xMax), :);
figure
imshow(MartianSurface)
hold on;
plot(hiRes_x, hiRes_y, 'r-', 'LineWidth', 2);

% Plot waypoints as red dots
plot(hiRes_x, hiRes_y, 'ro', 'MarkerFaceColor', 'r');
% Rectangle dimensions
width = xMax - xMin;
height = yMax - yMin;

% Draw the rectangle
rectangle('Position', [xMin, yMin, width, height], ...
          'EdgeColor', 'g', 'LineWidth', 2, 'LineStyle', '--');
% Optional: visualize
figure;
imshow(localPatch);
title('Local Patch Between Waypoints');
