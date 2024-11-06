% Parameters
mu_x = 2;        % Mean of x
sigma_x = 0.5;   % Standard deviation of x
mu_y = 1;        % Mean of y
sigma_y = 0.3;   % Standard deviation of y

% Function definition
f = @(x, y) x.^2 + 3*y;

%% 1. First-order Taylor Series Approximation

% Compute f at the mean values
f_mu = f(mu_x, mu_y);

% Compute partial derivatives at the mean values
df_dx = 2 * mu_x;    % Partial derivative of f with respect to x at (mu_x, mu_y)
df_dy = 3;           % Partial derivative of f with respect to y is constant

% Variance propagation formula
sigma_f_taylor = sqrt((df_dx^2 * sigma_x^2) + (df_dy^2 * sigma_y^2));

fprintf('Taylor Series Approximation:\n');
fprintf('Mean of f: %.2f\n', f_mu);
fprintf('Standard deviation of f: %.2f\n\n', sigma_f_taylor);

%% 2. Monte Carlo Error Propagation

% Number of samples
num_samples = 10000;

% Generate random samples for x and y
x_samples = normrnd(mu_x, sigma_x, [num_samples, 1]);
y_samples = normrnd(mu_y, sigma_y, [num_samples, 1]);

% Compute f for each sample
f_samples = f(x_samples, y_samples);

% Calculate mean and standard deviation of f from the samples
f_mean_mc = mean(f_samples);
f_std_mc = std(f_samples);

fprintf('Monte Carlo Simulation:\n');
fprintf('Mean of f: %.2f\n', f_mean_mc);
fprintf('Standard deviation of f: %.2f\n', f_std_mc);
