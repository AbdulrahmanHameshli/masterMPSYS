    %% Part a)
clc; clear all;

% Generate random noise
w = normrnd(0, 0.5, 1000);

% Define possible values for theta and their probabilities
values = [-1, 1];
probs = [0.5, 0.5];

% Sample theta values according to probabilities
theta = randsample(values, 1000, true, probs);

% Generate observed data
y = theta + w;

% Plot histogram of observed data
histogram(y, 'Normalization', 'pdf');
hold on;

% Define parameters for the true distribution of y
sigma2 = 0.5^2;
y_range = linspace(-5, 5, 10000);

% Calculate the true probability density function of y
p_y = 0.5 * (1/sqrt(2*pi*sigma2)) * exp(-(y_range - 1).^2 / (2*sigma2)) + ...
      0.5 * (1/sqrt(2*pi*sigma2)) * exp(-(y_range + 1).^2 / (2*sigma2));

% Plot the true distribution
plot(y_range, p_y, 'LineWidth', 2);
xlabel('y');
ylabel('p(y)');
title('Probability Density Function p(y)');
grid on;

%% Parts c and d)
% Given variables
sigma_sq = 0.5^2;              % Variance of the noise w
prior_theta_minus_1 = 0.5;     % Prior probability for theta = -1
prior_theta_1 = 0.5;           % Prior probability for theta = 1

% Observed data y
observed_y = 0.7;

% Calculate likelihoods p(y | theta = -1) and p(y | theta = 1)
likelihood_y_given_theta_minus_1 = (1 / sqrt(2*pi*sigma_sq)) * exp(-(observed_y + 1).^2 / (2*sigma_sq));
likelihood_y_given_theta_1 = (1 / sqrt(2*pi*sigma_sq)) * exp(-(observed_y - 1).^2 / (2*sigma_sq));

% Calculate total probability of observing y: p(y)
total_probability_y = 0.5 * likelihood_y_given_theta_minus_1 + 0.5 * likelihood_y_given_theta_1;

% Calculate posterior probabilities for theta = -1 and theta = 1
posterior_theta_minus_1 = (likelihood_y_given_theta_minus_1 * prior_theta_minus_1) / total_probability_y;
posterior_theta_1 = (likelihood_y_given_theta_1 * prior_theta_1) / total_probability_y;

% Display the results
fprintf('Posterior probability for theta = -1 given y: %f\n', posterior_theta_minus_1);
fprintf('Posterior probability for theta = 1 given y: %f\n', posterior_theta_1);

%% Part e)
% Calculate the mean of theta using posterior probabilities
theta_mean = posterior_theta_1 * 1 + posterior_theta_minus_1 * (-1);

% Display the mean of theta
fprintf('Mean of theta given y: %f\n', theta_mean);

