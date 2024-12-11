function [poseOut, covOut] = propagateut(x, C, b)
   
 
    
    % Unscented Transform for Covariance Propagation
    n = length(x); % State dimension
    w0 = 1/3; % Weight for the first sigma point
    wi = (1 - w0) / (2 * n); % Weights for other sigma points
    gamma = sqrt(n / (1 - w0)); % Scaling parameter

    % Calculate sigma points
    cholC = chol(C, 'lower'); % Cholesky decomposition
    sigmaPoints = [x, x + gamma * cholC, x - gamma * cholC];
    
    % Transform sigma points
    transformedPoints = zeros(3, 2 * n + 1);
    for i = 1:size(sigmaPoints, 2)
        spPose = sigmaPoints(1:3, i);
        spSL = sigmaPoints(4, i);
        spSR = sigmaPoints(5, i);

        dist = (spSL + spSR) / 2;
        deltaTheta = (spSR - spSL) / b;

        if abs(deltaTheta) < 1e-6
            % Straight movement
            Thetaout = spPose(3);
            Xout = spPose(1) + cos(Thetaout) * dist;
            Yout = spPose(2) + sin(Thetaout) * dist;
        else
            % Curved movement
            Thetaout = spPose(3) + deltaTheta;
            Xout = spPose(1) + (sin(Thetaout) - sin(spPose(3))) * (dist / deltaTheta);
            Yout = spPose(2) - (cos(Thetaout) - cos(spPose(3))) * (dist / deltaTheta);
        end

        transformedPoints(:, i) = [Xout; Yout; Thetaout];
    end

    % Compute mean and covariance of the transformed points
    poseOut = w0 * transformedPoints(:, 1);
    for i = 2:size(transformedPoints, 2)
        poseOut = poseOut + wi * transformedPoints(:, i);
    end

    covOut = w0 * (transformedPoints(:, 1) - poseOut) * (transformedPoints(:, 1) - poseOut)';
    for i = 2:size(transformedPoints, 2)
        covOut = covOut + wi * (transformedPoints(:, i) - poseOut) * (transformedPoints(:, i) - poseOut)';
    end
end
