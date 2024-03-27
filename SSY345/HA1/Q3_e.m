% clc
% clear
% 
% x = rand(1000);
% % x = unifpdf(x);
% % r = normpdf(normrnd(0,1, [1000,1000]));
% r = normrnd(0,1, [1000,1000]);
% h = @(x)(2*x^2);
% y = h(x) + r;
% 
% 
% figure(1);
% histogram(x)
%     
% figure(2);
% histogram(h(x))
% 
% figure(3);
% histogram(r)
% 
% figure(4)
% histogram(y)
% 
% disp("mean")
%     
% mean(var(y))
% mean(var(h(x)))
% 
% 
% % mean(mean(h(x)))
% % mean(mean(y))
% 
% [Y_given_X, Y_values] = ksdensity(y, x);






% Define a range of inputs
inputs = linspace(-10, 10, 100000); % Generate 100 equally spaced input values from -10 to 10

% Compute outputs using the deterministic function
outputs = deterministic_function(inputs);

% Plot the deterministic function
figure;
plot(inputs, outputs, 'b', 'LineWidth', 2);
xlabel('Input');
ylabel('Output');
title('Deterministic Function: y = x^2');

% Compute mean and variance of inputs (just to demonstrate)
mean_input = mean(outputs);
variance_input = var(outputs);

disp(['Mean of inputs: ', num2str(mean_input)]);
disp(['Variance of inputs: ', num2str(variance_input)]);

histogram(outputs)
% Define a deterministic function
function y = deterministic_function(x)
    y = sin(x).^2; % Example of a nonlinear deterministic function
end