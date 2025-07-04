
    d=2;
    % Extract last two points
    x1 = roverPath_x(end-1);
    y1 = roverPath_y(end-1);
    x2 = roverPath_x(end);
    y2 = roverPath_y(end);

    % Direction vector from second last to last point
    dx = x2 - x1;
    dy = y2 - y1;

    % Normalize the direction vector
    len = hypot(dx, dy);
    if len == 0
        error('The last two rover path points are the same — undefined direction.');
    end
    dx = dx / len;
    dy = dy / len;

    % Move forward from the last point by distance d
    x_sample = x2 + d * dx;
    y_sample = y2 + d * dy;
