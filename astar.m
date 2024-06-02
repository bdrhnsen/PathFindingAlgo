% A* Pathfinding Algorithm
function [route, numExpanded] = astar(input_map, start_coords, dest_coords)
    % Initialize variables
    [rows, cols] = size(input_map);
    start = sub2ind([rows, cols], start_coords(1), start_coords(2));
    goal = sub2ind([rows, cols], dest_coords(1), dest_coords(2));

    % Initialize open and closed lists
    openSet = [start];
    cameFrom = NaN(rows, cols);
    gScore = Inf(rows, cols);
    gScore(start) = 0;
    fScore = Inf(rows, cols);
    fScore(start) = heuristic(start_coords, dest_coords);

    closedSet = false(rows, cols);
    numExpanded = 0;

    while ~isempty(openSet)
        % Get node in openSet with lowest fScore
        [~, idx] = min(fScore(openSet)); % choose the node with lowest fscore
        current = openSet(idx);
        [current_row, current_col] = ind2sub([rows, cols], current);

        % If goal is reached, reconstruct path
        if current == goal
            route = reconstructPath(cameFrom, current);
            return;
        end

        % Remove current from openSet and add to closedSet
        openSet(idx) = [];
        closedSet(current) = true;

        % Get neighbors
        neighbors = getNeighbors(current_row, current_col, rows, cols);

        for i = 1:size(neighbors, 1)
            neighbor_row = neighbors(i, 1);
            neighbor_col = neighbors(i, 2);
            neighbor = sub2ind([rows, cols], neighbor_row, neighbor_col);

            % Skip if neighbor is in closedSet or is an obstacle
            if closedSet(neighbor) || input_map(neighbor)
                continue;
            end

            % Calculate tentative gScore
            tentative_gScore = gScore(current) + 1;
            if tentative_gScore < gScore(neighbor)
                cameFrom(neighbor) = current;
                gScore(neighbor) = tentative_gScore;
                fScore(neighbor) = gScore(neighbor) + heuristic([neighbor_row, neighbor_col], dest_coords);

                if ~ismember(neighbor, openSet)
                    openSet = [openSet; neighbor];
                    % Visualize the neighbor being added to openSet
                    plot(neighbor_col, neighbor_row, 'co', 'MarkerSize', 10, 'MarkerFaceColor', 'c');
                    pause(0.1); % Pause for visualization
                end
            end
        end
        numExpanded = numExpanded + 1;

        % Visualize the current node being processed
        plot(current_col, current_row, 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
        pause(0.01); % Pause for visualization
    end

    % If openSet is empty and goal was not reached
    route = [];
end
% Heuristic function for A* (Manhattan distance)
function h = heuristic(a, b)
    h = abs(a(1) - b(1)) + abs(a(2) - b(2));
end


% Get neighbors of a node
function neighbors = getNeighbors(row, col, numRows, numCols)
    directions = [0 1; 1 0; 0 -1; -1 0];
    neighbors = [];
    for i = 1:size(directions, 1)
        neighbor_row = row + directions(i, 1);
        neighbor_col = col + directions(i, 2);
        if neighbor_row > 0 && neighbor_row <= numRows && neighbor_col > 0 && neighbor_col <= numCols
            neighbors = [neighbors; neighbor_row, neighbor_col];
        end
    end
end

% Reconstruct path from cameFrom map
function route = reconstructPath(cameFrom, current)
    route = [];
    while ~isnan(cameFrom(current))
        route = [current; route];
        current = cameFrom(current);
    end
    route = [current; route];
end

