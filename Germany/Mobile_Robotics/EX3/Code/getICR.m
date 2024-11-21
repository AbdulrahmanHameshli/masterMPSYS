function ICR = getICR(xr, wheels)
    % xr: the pose of the robot [x, y, theta]
    % wheels: a structure array containing wheel information
    %         Each wheel has fields:
    %           - wheels(i).pose: [x, y, theta] in the robot's frame
    %           - wheels(i).type: type of the wheel
    % ICR: the position of the ICR in world coordinates, or [NaN, NaN] if invalid
    
    n = length(wheels);
    A = [];
    b = [];

    for i = 1:n
        type = wheels(i).type;


        if type == 2 || type == 3
            continue;
        end

   
        wheel_pose = wheels(i).pose;
        x_w = wheel_pose(1);
        y_w = wheel_pose(2);
        theta_w = wheel_pose(3);

   
        nx = -sin(theta_w);
        ny = cos(theta_w);

   
        A = [A; nx, ny];
        b = [b; nx * x_w + ny * y_w];
    end

       disp(['Rank of A: ', num2str(rank(A))]);
    if rank(A) < 2
        ICR = [NaN, NaN];
        return;
    end

   
    ICR_local = A \ b;

   
    tolerance = 1e-6; 
    valid = true; 
    for i = 1:n
        type = wheels(i).type;
        if type == 2 || type == 3
            continue;
        end

        wheel_pose = wheels(i).pose;
        x_w = wheel_pose(1);
        y_w = wheel_pose(2);
        theta_w = wheel_pose(3);
        nx = -sin(theta_w);
        ny = cos(theta_w);
        distance = abs(nx * ICR_local(1) + ny * ICR_local(2) - (nx * x_w + ny * y_w));
        
        if distance > tolerance
            valid = false;
            break;
        end
    end
    if ~valid
        ICR = [NaN, NaN];
        return;
    end

    x_r = xr(1);
    y_r = xr(2);
    theta_r = xr(3);
    R = [cos(theta_r), -sin(theta_r); sin(theta_r), cos(theta_r)];
    ICR = transpose(R * ICR_local + [x_r; y_r]);
end
