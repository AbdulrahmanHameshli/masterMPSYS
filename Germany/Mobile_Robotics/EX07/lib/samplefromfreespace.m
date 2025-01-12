% This function computes a set of random samples with configurable
% min and max values for x, y, and theta.
% Inputs:
%   min_x: minimum value for x position
%   max_x: maximum value for x position
%   min_y: minimum value for y position
%   max_y: maximum value for y position
%   min_theta: minimum value for theta
%   max_theta: maximum value for theta
%   dim: number of required samples

% Outputs:
%   samples: the randomly chosen samples

function samples = samplefromfreespace(min_x, max_x, min_y, max_y, min_theta, max_theta, dim)

% Generate random samples for x, y, and theta within specified ranges
x_samples = min_x + (max_x - min_x) * rand(1, dim);
y_samples = min_y + (max_y - min_y) * rand(1, dim);
theta_samples = min_theta + (max_theta - min_theta) * rand(1, dim);

% Combine into a single matrix
samples = [x_samples; y_samples; theta_samples];

% Normalize the theta values
samples(3, :) = normalizeAngle(samples(3, :));

end

function o = normalizeAngle(th)
    o = atan2(sin(th), cos(th));
end
