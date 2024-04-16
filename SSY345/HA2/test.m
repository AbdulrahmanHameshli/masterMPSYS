% Clear command window and workspace
clc; 
clear;

%% Section A: Generate Data
% Define parameters
x_0 = 2;
P_0 = 8;
A = 1;
Q = 1.5;
N = 35; 
H = 1;
R = 3;

% Generate linear state sequence and measurements
X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);

% Display sizes of X and Y
disp('Size of X:');
disp(size(X));
disp('Size of Y:');
disp(size(Y));

% Plot data
plotData(X, Y, N);

%% Section B: Kalman Filtering
[X_Filtered, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
std = sqrt(reshape(P, 1, N));

% Plot filtered data and error densities
plotFilteredData(X, Y, X_Filtered, std);

%% Section C: Variation of Initial State
[X_Filtered_C, ~] = kalmanFilter(Y, 12, P_0, A, Q, H, R);
plotFilteredData(X, Y, X_Filtered_C, std);

%% Section D: Prediction and Update
plotPredictionAndUpdate(X_Filtered, std, Y, R, X(13));

%% Section 2: Additional Analysis
% Load sensor measurements
data = load('SensorMeasurements.mat');

% Analyze sensor data
analyzeSensorData(data);

%% Functions
function plotData(X, Y, N)
    figure;
    hold on;
    plot(0:N, X, 'LineWidth', 3);
    plot(1:N, Y, 'LineWidth', 1);
    legend('Process x', 'Measurements y');
    title('Linear Gaussian state space model');
    xlabel('N');
end

function plotFilteredData(X, Y, X_Filtered, std)
    figure;
    hold on;
    plot(0:size(X, 2)-1, X, 'LineWidth', 2);
    plot(1:size(Y, 2), Y, 'LineWidth', 2);
    plot(1:size(X_Filtered, 2), X_Filtered, 'LineWidth', 2);
    plot(1:size(X_Filtered, 2), X_Filtered + 3*std, ':', 'LineWidth', 2, 'Color', 'k');
    plot(1:size(X_Filtered, 2), X_Filtered - 3*std, ':', 'LineWidth', 2, 'Color', 'k');
    legend('Process x', 'Measurements y', 'Kalman filtered measurements');
    title('Kalman Filter Results');
    xlabel('N');
    grid on;
end

function plotPredictionAndUpdate(X_Filtered, std, Y, R, X_k)
    figure;
    sam = -15:0.01:0;
    % Prior
    prior = normpdf(sam, X_Filtered(end), std(end));
    % Prediction
    [predicted_X, predicted_P] = linearPrediction(X_Filtered(end), std(end)^2, 1, 1.5);
    prediction = normpdf(sam, predicted_X, sqrt(predicted_P));
    % Measurement
    YK = normpdf(sam, Y(end), R);
    % Update
    [Update_X, Update_P] = linearUpdate(predicted_X, predicted_P, Y(end), 1, R);
    update = normpdf(sam, Update_X, sqrt(Update_P));
    plot(sam, prior);
    plot(sam, prediction);
    plot(sam, YK);
    plot(sam, update);
    legend('prior:', 'prediction:', 'y', 'update');
    title('Prediction and Update');
    grid on;
    xlim([-15 0]);
    ylim([0 0.35]);
end

function analyzeSensorData(data)
    Train_0 = data.CalibrationSequenceVelocity_v0;
    Train_10 = data.CalibrationSequenceVelocity_v10;
    Train_20 = data.CalibrationSequenceVelocity_v20;

    % Calculate mean and variance
    mean_Train_20 = mean(Train_20);
    C1 = 1.10264;
    C2 = 1.102265;
    C = (C1 + C2) / 2;
    var2 = var(Train_10 / C);
    var3 = var(Train_20 / C);

    % Plot sensor data
    figure;
    subplot(2, 1, 1);
    plot(Train_10);
    hold on;
    illust_data1 = C1 * (10 + mvnrnd(mean(0), var2, 2000)');
    plot(illust_data1);
    subplot(2, 1, 2);
    plot(Train_20);
    hold on;
    illust_data2 = C2 * (20 + mvnrnd(mean(0), var3, 2000)');
    plot(illust_data2);
end



%% genLinearStateSequenceC
function X = genLinearStateSequence(x_0, P_0, A, Q, N)

n = size(x_0, 1);

X = zeros(n, N+1);
X(:, 1) = mvnrnd(x_0, P_0)';

for i = 2:N+1
    X(:, i) = (A * X(:, i-1)) + mvnrnd(zeros(n,1), Q)';
end
end

%% genLinearMeasurementSequence
function Y = genLinearMeasurementSequence(X, H, R)

[n, Np1] = size(X);
m = size(H, 1);
N = Np1 - 1;
Y = zeros(m, N);

for k = 1:N
    x_k = X(:, k+1);
    v_k = mvnrnd(zeros(m, 1), R)';
    Y(:, k) = H * x_k + v_k;
end
end

%% KalmanFilter
function [X, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R)
N = size(Y,2);

n = length(x_0);
m = size(Y,1);
x = zeros(n,N);
P = zeros(n,n,N);
   

x_pred = x_0;
P_pred = P_0;

for k = 1:N
    x_pred = A * x_pred; 
    P_pred = A * P_pred * A' + Q; % Predicted state covariance
    
    K = P_pred * H' / (H * P_pred * H' + R);
    y_k = Y(:, k) - H * x_pred;

    x_pred = x_pred + K * y_k;
    P_pred = (eye(n) - K * H) * P_pred;

    X(:, k) = x_pred;
    P(:, :, k) = P_pred;
end

end
%% linearPrediction
function [x, P] = linearPrediction(x, P, A, Q)
x = A*x; 
P = A*P*A' + Q;  
end
%% linearUpdate
function [x, P] = linearUpdate(x, P, y, H, R)

K = P*H'*inv((H*P*H'+R));
y_hat = -H*x  + y ;

x = x + K*y_hat;
P = P - K*H*P;


end