% Mobile Robotics Exercise
%
% EKF Localization
%
% Measurement function
%
% Parameters:
%   - state: robot state in the global frame (3x1) [xPos; yPos; state(3)]
%   - globalBeacon: beacon position in the global frame (2x1) [xPos; yPos]
% 
% Returns:
%   - transformedGlobalBeacon: position of the beacon in robot frame (2x1)
%                            [xPos; yPos]
%   - measurementFunctionJacobian: jacobian of the measurement function

function [transformedGlobalBeacon, measurementFunctionJacobian] = measurementfunction(state, globalBeacon)

delta = (globalBeacon - [state(1); state(2)]);
delta_x = delta(1);
delta_y = delta(2);
transformedGlobalBeacon = [cos(state(3)) sin(state(3)) ; -sin(state(3)) cos(state(3)) ] * [delta_x; delta_y];

	% TODO compute the measurement prediction and the jacobian of the
    % measurement function
measurementFunctionJacobian = [
        -cos(state(3)), -sin(state(3)), -sin(state(3)) * delta_x + cos(state(3)) * delta_y;
         sin(state(3)), -cos(state(3)), -cos(state(3)) * delta_x - sin(state(3)) * delta_y
    ];

end
