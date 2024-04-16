clc, clear all;
%% A
x_0 = 2;
P_0 = 8;
A = 1;
Q = 1.5;
N = 35; 
H=1;
R = 3;

X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);

size(X)
size(Y)

figure()
hold on
plot(0:N, X,'Linewidth', 3)
plot(1:N, Y,'linewidth', 1)
legend('process x','Measurements y')
title('Linear Gaussian state space model')
xlabel('N');


%% B
[X_Filtred, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);

std = sqrt(reshape(P,1,35));


% Plot the results
figure()
hold on
grid on
plot(0:N, X,'Linewidth', 2)
plot(1:N, Y,'linewidth', 2)
plot(1:N, X_Filtred,'linewidth', 2)
plot(1:N, X_Filtred+(3*std),':','linewidth', 2,'color','k')
plot(1:N, X_Filtred-(3*std),':','linewidth', 2,'color','k')
legend('process x','Measurements y','Kalman filtered measurements')

% Plot error densities
figure;
counter = 0;
i = [1, 2, 4, 30];
for n = 1:4
    counter = counter + 1;
    err = (X(i(n)) - X_Filtred(i(n)))^2;
    a = mvnrnd(0, err, 10000);
    subplot(4, 1, counter);
    histogram(a, 75, 'Normalization', 'pdf');
    legend(sprintf('i=%d', i(n))); % Adding legend with i value
    xlabel('Error');
    ylabel('Density');
    title(sprintf('Error Density for i=%d', i(n))); % Adding title
    hold on;
end

%% C


[X_Filtred, P] = kalmanFilter(Y, 12, P_0, A, Q, H, R);

% Plot the results
figure()
hold on
grid on
plot(0:N, X,'Linewidth', 2)
plot(1:N, Y,'linewidth', 2)
plot(1:N, X_Filtred,'linewidth', 2)
plot(1:N, X_Filtred+(3*std),':','linewidth', 2,'color','k')
plot(1:N, X_Filtred-(3*std),':','linewidth', 2,'color','k')
legend('process x','Measurements y','Kalman filtered measurements')
%% D

figure()
sam = -15:0.01:0;
k = 13;

% p(x-1|y1:k-1)
prior = normpdf(sam,X_Filtred(k-1),std(k-1));

% P(x(k)|y1:k-1)
[predicted_X, predicted_P] = linearPrediction(X_Filtred(k-1), std(k-1)^2, A, Q);
prediction = normpdf(sam,predicted_X,sqrt(predicted_P));

%Y(k)
YK = normpdf(sam,Y(k),R);


% p(x(k)|y1:k)
[Uppdate_X , Uppdate_P]  = linearUpdate(predicted_X, predicted_P, Y(k), H, R);
Uppdate = normpdf(sam,Uppdate_X,sqrt(Uppdate_P));


hold on; grid on;
xlim([-15 0])
ylim([0 0.35])
plot(sam,prior);
plot(sam,prediction);
plot(sam,YK);
plot(sam,Uppdate);

legend('prior:','prediction: ','y','update')




%-------------------------------------------------------------------------------2------------------------------------------------------------------------------
%%
% Load sensor measurements
data = load('SensorMeasurements.mat');

% Extract velocity data for different calibration sequences
Train_0 = data.CalibrationSequenceVelocity_v0;
Train_10 = data.CalibrationSequenceVelocity_v10;
Train_20 = data.CalibrationSequenceVelocity_v20;

% Calculate the mean of Train_20
mean_train_20 = mean(Train_20);

% Calculate the average calibration factor
C1 = 1.10264;
C2 = 1.102265;
C = (C1 + C2) / 2;

% Compute variance normalized by the calibration factor
var2 = var(Train_10 / C);
var3 = var(Train_20 / C);

% Generate illustrated data based on the calibration factors
illust_data1 = C1 * (10 + mvnrnd(0, var2, 2000)');
illust_data2 = C2 * (20 + mvnrnd(0, var3, 2000)');

% Plot the sensor measurements and illustrated data
figure()
subplot(2,1,1)
plot(Train_10)
hold on
plot(illust_data1)
title('Sensor Measurements and Illustrated Data for Velocity 10')
legend('Measured Data', 'Illustrated Data')
xlabel('Time')
ylabel('Velocity')
grid on

subplot(2,1,2)
plot(Train_20)
hold on
plot(illust_data2)
title('Sensor Measurements and Illustrated Data for Velocity 20')
legend('Measured Data', 'Illustrated Data')
xlabel('Time')
ylabel('Velocity')
grid on

% Kalman filter for motion model CV
h = 0.2;
Y = Generate_y_seq;
Y = Y(:,1:2:end);
Y(2,:) = Y(2,:) ./ C;
time = h:h:200;

x_0 = [0; 0];
P_0 = [10 10; 10 10];
A_cv = [1 h; 0 1];
Q = [0 0; 0 0.0001];
H = [1 0; 0 1];
R = [1 0; 0 mean([var2, var3])];

[X_cv, ~] = kalmanFilter(Y, x_0, P_0, A_cv, Q, H, R);

% Plot results
figure()
subplot(2,1,1)
plot(time, X_cv(1,:), 'linewidth', 3)
hold on
plot(time, Y(1,:), ':k', 'linewidth', 2, 'Color', 'red')
title('Motion Model CV - Position')
legend('Estimated Position', 'Measured Position')
xlabel('Time [s]')
ylabel('Position [m]')
grid on

subplot(2,1,2)
plot(time, X_cv(2,:), 'linewidth', 3)
hold on
plot(time, Y(2,:), ':k', 'linewidth', 2, 'Color', 'red')
title('Motion Model CV - Velocity')
legend('Estimated Velocity', 'Measured Velocity')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
grid on

% Kalman filter for motion model CA
h = 2 * 0.1;
Y = Generate_y_seq;
x_0 = [0; 0; 0];
P_0 = eye(3);
A_ca = [1 h h*h/2; 0 1 h; 0 0 1];
Q = [0 0 0; 0 0 0; 0 0 0.0001];
H = [1 0 0; 0 1 0];
R = [3 0; 0 mean([var2, var3])];

Y = Y(:,1:2:end);
Y(2,:) = Y(2,:) ./ C;
time = h:h:200;
[X_ca, ~] = kalmanFilter(Y, x_0, P_0, A_ca, Q, H, R);

% Plot results
figure()
subplot(2,1,1)
plot(time, X_ca(1,:), 'linewidth', 3, 'Color', 'red')
hold on
plot(time, Y(1,:), ':k', 'linewidth', 2)
title('Motion Model CA - Position')
legend('Estimated Position', 'Measured Position')
xlabel('Time [s]')
ylabel('Position [m]')
grid on

subplot(2,1,2)
plot(time, X_ca(2,:), 'linewidth', 3)
hold on
plot(time, Y(2,:), ':k', 'linewidth', 2, 'Color', 'red')
title('Motion Model CA - Velocity')
legend('Estimated Velocity', 'Measured Velocity')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
grid on

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
