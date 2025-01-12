% % Mobile Robotics Exercise
% %
% % EKF Localization
% %
% % Observation prediction and data association
% %
% % Parameters:
% %   - state: robot state in the global frame (3x1) [xPos; yPos; theta]
% %   - C: robot state uncertainty covariance matrix
% %   - localBeacons: observed beacon positions in the robot frame
% %                   (2xn) [xPos; yPos]
% %   - globalMap: datastructure containing global map information
% %   - config: configuration data for the robot and the simulation
% % 
% % Returns:
% %   - totalInnovation: stacked innovation
% %   - totalH: stacked jacobian of the measurement function
% %   - totalR: stacked measurement uncertainty
% 
% function [totalInnovation, totalH, totalR] = predictandmatch(state, C, localBeacons, globalMap, config)
% 
%     
%     % TODO implement data assiciation using the measurementfunction() and
%     % calculate stacked innovation, stacked measurement jacobian and
%     % stacked measurement uncertainty
%     
%     
% 
% 
% end
% 


function [totalInnovation, totalH, totalR] = predictandmatch(state, C, localBeacons, globalMap, config)
% EKF Localization: Observation prediction and data association
%
% Parameters:
%   - state: Robot state in the global frame (3x1) [xPos; yPos; theta]
%   - C: Robot state uncertainty covariance matrix
%   - localBeacons: Observed beacon positions in the robot frame (2xn) [xPos; yPos]
%   - globalMap: Global beacon positions in the global frame (2xm) [xPos; yPos]
%   - config: Configuration data (e.g., measurement noise)
%
% Returns:
%   - totalInnovation: Stacked innovation vector
%   - totalH: Stacked Jacobian matrix
%   - totalR: Stacked measurement uncertainty

% Define constants
    BEACON = 8;
    % Beacons
    M = [
         2,  -2, 2,  -2, BEACON;
         1.4,  2.4, 1.4,  2.4, BEACON;
    %
         1,  7.4, 1,  7.4, BEACON; %
    %
         -4.6,   0.65, -4.6,   0.65, BEACON;
         -4.6,   4.65, -4.6,   4.65, BEACON;
    %
         -13.5, 4.65, -13.5, 4.65, BEACON;
         -13.5, 0.65, -13.5, 0.65, BEACON;
    
         -4.4, -2.3, -4.4, -2.3, BEACON;
         -10.6, -2.3, -10.6, -2.3, BEACON;
         -9.8, 0.65, -9.8, 0.65, BEACON;
         -9.8, 4.65, -9.8, 4.65, BEACON;
         
         ];
    % Extract rows corresponding to BEACONS
    beaconRows = M(M(:,5) == BEACON, :);
    
    % Extract the x and y coordinates of the beacons
    globalBeaconPositions = [beaconRows(:,1)'; beaconRows(:,2)'];

    

    % Initialize outputs
    totalInnovation = [];
    totalH = [];
    totalR = [];

    % Measurement noise covariance matrix (2x2)
    R = config.CBEAC;  % Measurement noise covariance

    % Loop over observed local beacons
    for i = 1:size(localBeacons, 2)
        z_obs = localBeacons(:, i); % Observed beacon position in robot frame
        
        minMahalanobis = inf;      % Initialize minimum Mahalanobis distance
        bestInnovation = [];       % Innovation for the best match
        bestH = [];                % Jacobian for the best match
        bestR = [];                % Measurement noise for the best match
        
        % Loop over all global beacons
        for j = 1:size(globalBeaconPositions, 2)
            globalBeacon = globalBeaconPositions(:, j); % Global beacon position
            
            % Predict the beacon position in robot frame using measurementfunction
            [z_pred, H] = measurementfunction(state, globalBeacon);
            
            % Calculate innovation
            innovation = z_obs - z_pred;
            
            % Compute Mahalanobis distance
            S = H * C * H' + R; % Innovation covariance
            mahalanobisDist = innovation' / S * innovation;
            
            % Check if this beacon is the best match
            if mahalanobisDist < minMahalanobis
                minMahalanobis = mahalanobisDist;
                bestInnovation = innovation;
                bestH = H;
                bestR = R;
            end
        end

        % Check statistical compatibility using chi-square threshold
        threshold = chi2inv(0.95, 2); % 95% confidence, 2 DOF
        if minMahalanobis <= threshold
            % Append best match data to the stacked results
            totalInnovation = [totalInnovation; bestInnovation];
            totalH = [totalH; bestH];
            totalR = blkdiag(totalR, bestR);
            disp("Match")
        end
    end
end
