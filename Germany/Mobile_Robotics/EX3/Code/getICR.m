function ICR = getICR(xr, wheels)
    % xr: the pose of the robot [x, y, theta]
    % wheels: a structure array containing wheel information
    %         Each wheel has fields:
    %           - wheels(i).pose: [x, y, theta] in the robot's frame
    % ICR: the position of the ICR in world coordinates, or [NaN, NaN] if it does not exist

    n = length(wheels);
        A = [];
    b = [];

   for i = 1:n
        type = wheels(i).type;
    
        % Check if the type is 2 or 3
        if type == 2 || type == 3
            break;
        else
            % Extract wheel pose
            wheel_pose = wheels(i).pose;
            x_w = wheel_pose(1);
            y_w = wheel_pose(2);
            theta_w = wheel_pose(3);
            
            nx = -sin(theta_w);
            ny = cos(theta_w);
            
            A = [A; nx, ny];
            b = [b; nx * x_w + ny * y_w];
        end
    end

    if rank(A) < 2 
        ICR = [NaN, NaN]; 
    else
        ICR_local = A \ b;
        x_r = xr(1);
        y_r = xr(2);
        theta_r = xr(3);
        
        R = [cos(theta_r), -sin(theta_r); sin(theta_r), cos(theta_r)];
        ICR = transpose(R * ICR_local + [x_r; y_r]);
     
    end
end

