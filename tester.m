    % Parameters
    input_map = false(10, 10); % Create a 10x10 grid with all free cells
    
    
    input_map(3, 5) = true;  % Add some obstacles
    input_map(4, 5) = true;
    input_map(5, 5) = true;
    input_map(6, 5) = true;
    input_map(7, 5) = true;
    input_map(8, 5) = true;
    
    input_map(2, 2) = true;
    input_map(2, 3) = true;

    input_map(9, 8) = true;
    input_map(9, 9) = true;

    input_map(1, 2) = true;
    input_map(1, 8) = true;

    input_map(10, 7) = true;

    start_coords = [1, 1];  % Start position
    dest_coords = [10, 10]; % Goal position

    % Plot the grid
    figure;
    subplot(1, 2, 1);
    imagesc(~input_map);
    colormap([1 1 1; 0 0 0]); % white for free cells, black for obstacles
    axis equal;
    hold on;
    plot(start_coords(2), start_coords(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(dest_coords(2), dest_coords(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title('A* Algorithm');

    % Run A* algorithm
    [route_a_star, numExpanded_a_star] = astar(input_map, start_coords, dest_coords);

    % Display the final path for A*
    if isempty(route_a_star)
        disp('A* Algorithm: No path found.');
    else
        [route_rows, route_cols] = ind2sub(size(input_map), route_a_star);
        plot(route_cols, route_rows, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        disp(['A* Algorithm: Number of nodes expanded: ', num2str(numExpanded_a_star)]);
    end

    % Plot the grid
    subplot(1, 2, 2);
    imagesc(~input_map);
    colormap([1 1 1; 0 0 0]); % white for free cells, black for obstacles
    axis equal;
    hold on;
    plot(start_coords(2), start_coords(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(dest_coords(2), dest_coords(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title('Dijkstra Algorithm');

    % Run Dijkstra's algorithm
    [route_dijkstra, numExpanded_dijkstra] = dijkstra(input_map, start_coords, dest_coords);

    % Display the final path for Dijkstra
    if isempty(route_dijkstra)
        disp('Dijkstra Algorithm: No path found.');
    else
        [route_rows, route_cols] = ind2sub(size(input_map), route_dijkstra);
        plot(route_cols, route_rows, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        disp(['Dijkstra Algorithm: Number of nodes expanded: ', num2str(numExpanded_dijkstra)]);
    end