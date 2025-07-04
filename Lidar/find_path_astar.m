function path = find_path_astar(grid_map, start_coords, goal_coords)
    % A* (A-star) pathfinding algorithm on a 2D grid.
    % INPUTS:
    %   grid_map: A 2D matrix where 0 is traversable and 1 is an obstacle.
    %   start_coords: A 1x2 vector [row, col] for the start position.
    %   goal_coords: A 1x2 vector [row, col] for the goal position.
    % OUTPUT:
    %   path: An N-by-2 matrix of [row, col] coordinates from start to goal,
    %         or an empty array if no path is found.

    [height, width] = size(grid_map);
    
    % The set of discovered nodes that are not yet evaluated.
    % Each entry is: [f_cost, g_cost, h_cost, row, col, parent_row, parent_col]
    open_list = [];
    
    % The set of nodes already evaluated.
    closed_list = false(height, width);
    
    % Heuristic cost (h_cost) map (Euclidean distance to goal)
    [cols, rows] = meshgrid(1:width, 1:height);
    h_cost_map = sqrt((rows - goal_coords(1)).^2 + (cols - goal_coords(2)).^2);

    % Check if start or goal are on an obstacle
    if grid_map(start_coords(1), start_coords(2)) == 1
        path = []; fprintf('A* Error: Start position is on an obstacle.\n'); return;
    end
    if grid_map(goal_coords(1), goal_coords(2)) == 1
        path = []; fprintf('A* Error: Goal position is on an obstacle.\n'); return;
    end

    g_cost = 0;
    h_cost = h_cost_map(start_coords(1), start_coords(2));
    f_cost = g_cost + h_cost;
    
    % Add the start node to the open list and a map for efficient lookups
    start_node = [f_cost, g_cost, h_cost, start_coords(1), start_coords(2), start_coords(1), start_coords(2)];
    open_list(1,:) = start_node;
    all_nodes_map = containers.Map('KeyType','char','ValueType','any');
    all_nodes_map(sprintf('%d,%d', start_coords(1), start_coords(2))) = start_node;
    
    path_found = false;

    while ~isempty(open_list)
        [~, min_idx] = min(open_list(:,1));
        current_node = open_list(min_idx,:);
        open_list(min_idx,:) = [];
        
        current_g = current_node(2);
        current_r = current_node(4);
        current_c = current_node(5);

        if closed_list(current_r, current_c)
            continue;
        end
        closed_list(current_r, current_c) = true;

        if current_r == goal_coords(1) && current_c == goal_coords(2)
            path_found = true;
            break; 
        end

        % Explore Neighbors (8-way connectivity)
        for dr = -1:1
            for dc = -1:1
                if dr == 0 && dc == 0; continue; end

                neighbor_r = current_r + dr;
                neighbor_c = current_c + dc;
                
                if neighbor_r < 1 || neighbor_r > height || neighbor_c < 1 || neighbor_c > width ...
                   || closed_list(neighbor_r, neighbor_c) || grid_map(neighbor_r, neighbor_c) == 1
                    continue;
                end

                new_g_cost = current_g + sqrt(dr^2 + dc^2);
                neighbor_key = sprintf('%d,%d', neighbor_r, neighbor_c);
                
                % --- THIS BLOCK IS NOW CORRECTED ---
                if isKey(all_nodes_map, neighbor_key)
                    % Step 1: Get the entire node data (the vector) from the map
                    existing_node = all_nodes_map(neighbor_key);
                    % Step 2: Index the vector to get the g_cost (the 2nd element)
                    if existing_node(2) <= new_g_cost 
                        continue; % The path we already found is better or the same
                    end
                end

                new_h_cost = h_cost_map(neighbor_r, neighbor_c);
                new_f_cost = new_g_cost + new_h_cost;
                new_node_data = [new_f_cost, new_g_cost, new_h_cost, neighbor_r, neighbor_c, current_r, current_c];
                all_nodes_map(neighbor_key) = new_node_data;
                open_list = [open_list; new_node_data];
            end
        end
    end

    % Reconstruct Path
    if path_found
        path = [];
        curr_key = sprintf('%d,%d', goal_coords(1), goal_coords(2));
        start_key = sprintf('%d,%d', start_coords(1), start_coords(2));
        
        while ~strcmp(curr_key, start_key)
            node_data = all_nodes_map(curr_key);
            path = [node_data(4), node_data(5); path];
            curr_key = sprintf('%d,%d', node_data(6), node_data(7));
            if ~isKey(all_nodes_map, curr_key)
                fprintf('Path reconstruction failed.\n'); path = []; return;
            end
        end
        path = [start_coords; path];
    else
        path = [];
        fprintf('A* Alert: No path could be found.\n');
    end
end