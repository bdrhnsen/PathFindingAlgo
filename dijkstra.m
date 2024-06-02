function [route, numExpanded] = dijkstra(input_map, start_coords, dest_coords)
    % Initialize variables
    [rows, cols] = size(input_map);
    start = sub2ind([rows, cols], start_coords(1), start_coords(2));
    goal = sub2ind([rows, cols], dest_coords(1), dest_coords(2));

    % Initialize distances and predecessors
    distances = Inf(rows, cols);
    distances(start) = 0;
    predecessors = NaN(rows, cols);

    % Initialize priority queue
    pq = PriorityQueue();
    pq.insert(start, 0);

    numExpanded = 0;

    while ~pq.isEmpty()
        current = pq.remove();
        [current_row, current_col] = ind2sub([rows, cols], current);

        % If goal is reached, reconstruct path
        if current == goal
            route = reconstructPath(predecessors, current, rows, cols);
            return;
        end

        % Get neighbors
        neighbors = getNeighbors(current_row, current_col, rows, cols);

        for i = 1:size(neighbors, 1)
            neighbor_row = neighbors(i, 1);
            neighbor_col = neighbors(i, 2);
            neighbor = sub2ind([rows, cols], neighbor_row, neighbor_col);

            % Skip if neighbor is an obstacle
            if input_map(neighbor)
                continue;
            end

            % Calculate tentative distance
            tentative_dist = distances(current) + 1;
            if tentative_dist < distances(neighbor)
                distances(neighbor) = tentative_dist;
                predecessors(neighbor) = current;

                if ~pq.contains(neighbor)
                    pq.insert(neighbor, distances(neighbor));
                else
                    pq.updatePriority(neighbor, distances(neighbor));
                end
            end
        end
        numExpanded = numExpanded + 1;

        
        % Visualize the current node being processed
        plot(current_col, current_row, 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
        pause(0.01); % Pause for visualization
    end

    % If no path found
    route = [];
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

% Reconstruct path from predecessors map
function route = reconstructPath(predecessors, current, numRows, numCols)
    route = [];
    while ~isnan(predecessors(current))
        route = [current; route];
        current = predecessors(current);
    end
    route = [current; route];
end
