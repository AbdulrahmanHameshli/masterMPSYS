function minDistance = closestobstacledistance(x, y, M)
    % closestObstacleDistance calculates the distance from a point (x, y)
    % to the closest obstacle in the map M.
    %
    % Inputs:
    %   x, y - Coordinates of the point.
    %   M    - Matrix representing the map. Each row defines an obstacle as:
    %          [p1x, p1y, p2x, p2y, type].
    %
    % Output:
    %   minDistance - The shortest distance to the closest obstacle.
    
    % Initialize minimum distance to a very large value
    minDistance = inf;
    
    % Iterate over each obstacle in the map
    for i = 1:size(M, 1)
        p1x = M(i, 1);
        p1y = M(i, 2);
        p2x = M(i, 3);
        p2y = M(i, 4);
        
        % Get the rectangle's corners
        corners = [
            p1x, p1y;
            p1x, p2y;
            p2x, p1y;
            p2x, p2y
        ];
        
        % Define the edges of the rectangle
        edges = [
            corners(1, :), corners(2, :);
            corners(1, :), corners(3, :);
            corners(2, :), corners(4, :);
            corners(3, :), corners(4, :);
        ];
        
        % Check the distance to each edge
        for j = 1:size(edges, 1)
            x1 = edges(j, 1);
            y1 = edges(j, 2);
            x2 = edges(j, 3);
            y2 = edges(j, 4);
            
            % Calculate the distance from the point to the current edge
            distance = pointtosegmentdistance(x, y, x1, y1, x2, y2);
            
            % Update the minimum distance if necessary
            if distance < minDistance
                minDistance = distance;
            end
        end
    end
end

function distance = pointtosegmentdistance(px, py, x1, y1, x2, y2)
    % pointtosegmentdistance calculates the shortest distance from a point
    % to a line segment defined by two points (x1, y1) and (x2, y2).
    
    % Vector from start to end of the segment
    dx = x2 - x1;
    dy = y2 - y1;
    % Vector from start to the point
    px_dx = px - x1;
    py_dy = py - y1;
    % Segment length squared
    segment_length_sq = dx^2 + dy^2;
    
    if segment_length_sq == 0
        % Start and end points are the same
        distance = sqrt(px_dx^2 + py_dy^2);
        return;
    end
    
    % Project the point onto the segment and clamp t to [0, 1]
    t = max(0, min(1, (px_dx * dx + py_dy * dy) / segment_length_sq));
    % Find the closest point on the segment
    closest_x = x1 + t * dx;
    closest_y = y1 + t * dy;
    % Return the distance to the closest point
    distance = sqrt((px - closest_x)^2 + (py - closest_y)^2);
end

