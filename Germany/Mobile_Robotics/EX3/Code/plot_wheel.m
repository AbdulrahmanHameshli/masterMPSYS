function plot_wheel(wheel_poses)
    % Plot settings
    figure;
    hold on;
    axis equal;
    grid on;
    xlabel('X-axis');
    ylabel('Y-axis');
    title('Wheel Positions and Orientations');
    
    % Define arrow properties
    arrow_length = 0.5;  % Length of the orientation arrow
    arrow_color = 'r';   % Color of the arrow
    
    % Plot each wheel
    for i = 1:size(wheel_poses, 1)
        x = wheel_poses(i, 1);
        y = wheel_poses(i, 2);
        theta = wheel_poses(i, 3);
        
        % Plot wheel position
        plot(x, y, 'ko', 'MarkerSize', 8, 'DisplayName', ['Wheel ', num2str(i)]);
        
        % Calculate arrow end point
        arrow_x = x + arrow_length * cos(theta);
        arrow_y = y + arrow_length * sin(theta);
        
        % Plot orientation arrow
        quiver(x, y, arrow_x - x, arrow_y - y, 0, 'Color', arrow_color, 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    end
    
    % Set plot limits
    xlim([-2, 8]);
    ylim([-2, 2]);
    
    % Add legend
    legend('show');
    
    hold off;
end
