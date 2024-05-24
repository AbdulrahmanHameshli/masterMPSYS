% 1.A
clear all;
close all;
clc;

N = 100000;
state_densities = {struct('x', [125; 125], 'P', diag([10, 5])), ...
                   struct('x', [-25; 125], 'P', diag([10, 5])), ...
                   struct('x', [60; 60], 'P', diag([10, 5]))};

s1 = [0; 100];
s2 = [100; 0];

R = diag([(0.1*pi)/180, (0.1*pi)/180]).^2;

% figure();
for i = 1:numel(state_densities)

    x = state_densities{i}.x;
    P = state_densities{i}.P;

    h = @(x) dualBearingMeasurement(x, s1, s2);

    [y_mean, P_y, y_s] = approxGaussianTransform(x, P, @(x) genNonLinearMeasurementSequence(x, h, R), N);
    
    disp(['Mean for state density ', num2str(i), ':']);
    disp(y_mean);
    disp(['Covariance for state density ', num2str(i), ':']);
    disp(P_y);
end

%%  B
close all
clc

for state_dens = 1:3
    % Initialize variables
    types = {'EKF', 'UKF', 'CKF'};
    
    % Define initial state and covariance based on state density
    if state_dens == 1
        x = [125; 125];
    elseif state_dens == 2
        x = [-25; 125];
    elseif state_dens == 3
        x = [60; 60];
    end
    P = diag([10^2, 5^2]);
    
    % Iterate over each type
    for type_index = 1:length(types)
        type = types{type_index};
        
        [mean_y, P_y] = analyticalapprox(x, P, h, R, type);

        state_dens
        disp(['Type: ', type]);
        mean_y
        P_y
        disp('---------------------------------------');
    end
end




%% C 
% Clear command window
clc;

% Define state densities and corresponding distributions
state_densities = {[125; 125], [-25; 125], [60; 60]};
P = diag([10^2, 5^2]);

% Set sensor positions
s1 = [0; 100];
s2 = [100; 0];

% Set measurement noise covariance
R = diag([(0.1*pi)/180, (0.1*pi)/180]).^2;

% Create handle for measurement model y = h(x) + R
h = @(x) dualBearingMeasurement(x, s1, s2);

% Define colors for the plots
colors = {'r', 'g', 'b'}; % Red for EKF, Green for UKF, Blue for CKF

% Loop over each state density
for state_index = 1:numel(state_densities)
    % Extract state density
    x = state_densities{state_index};
    
    % Compute approximate Gaussian transformation
    [y_mean, P_y, y_s] = approxGaussianTransform(x, P, @(x) genNonLinearMeasurementSequence(x, h, R), N);
    
    % Generate 2D ellipse for sample covariance
    level = 3;
    [xy_sample] = sigmaEllipse2D(y_mean, P_y, level, N);
    
    % Plot the samples with light blue color
    figure(state_index)
    hold on
    grid on
    scatter(y_s(1,:), y_s(2,:), 5, 'k', 'filled') % Sample points
    
    % Plot sample mean in light red
    scatter(y_mean(1), y_mean(2), 'filled', 'MarkerFaceColor', 'm') % Sample mean
    
    % Compute and plot sigma curves for sample covariance in light purple
    scatter(xy_sample(1,:), xy_sample(2,:), 'MarkerEdgeColor', 'm', 'LineWidth', 1.5) % Sample 3sigma curve
    
    % Analytical approximation
    types = {'EKF', 'UKF', 'CKF'};
    for i = 1:numel(types)
        % Choose type of KF
        type = types{i};

        % Compute the approximated mean and covariances analytically
        [mean_y, P_y] = analyticalapprox(x, P, h, R, type);

        % Generate 2D ellipse for analytical covariance
        [xy_analytical] = sigmaEllipse2D(mean_y, P_y, level, N);

        % Plot the analytical mean in light green
        scatter(mean_y(1), mean_y(2), 'filled', 'MarkerFaceColor', colors{i}) % Analytical mean
        
        % Plot sigma curve for analytical covariance
        scatter(xy_analytical(1,:), xy_analytical(2,:), 'MarkerEdgeColor', colors{i}, 'LineWidth', 1.5) % Analytical 3sigma curve
    end
    
    % Set plot title and legend
    title(['State Density ', num2str(state_index)])
    legend('Samples', 'Sample Mean', 'Sample 3\sigma-curve', ...
        'EKF Mean', 'EKF 3\sigma-curve', ...
        'UKF Mean', 'UKF 3\sigma-curve', ...
        'CKF Mean', 'CKF 3\sigma-curve', 'Location', 'best')
end
%% 2 A
% Clear command window and close all figures
clc;
close all;

% For case 1 define initial state and variance
x_0 = [0, 0, 20, 0, ((5*pi)/180)^2]';
P_0 = diag([10^2, 10^2, 2^2, (pi/180)^2, (pi/180)^2]);

sigma_v = 1;
sigma_w = pi/180;
sigma_phi1 = 2*pi/180;
sigma_phi2 = 2*pi/180;
Q = [0 0 0 0 0;
     0 0 0 0 0;
     0 0 (sigma_v)^2 0 0;
     0 0 0 0 0;
     0 0 0 0 (sigma_w)^2];
R = [sigma_phi1^2 0;
     0 sigma_phi2^2];

% Define two nonlinear functions
f = @(x) coordinatedTurnMotion(x, 1);
h = @(x) dualBearingMeasurement(x, [-200;100], [-200;-100]);

% Firstly get sequences of N=100
N = 100;
X_s = genNonLinearStateSequence(x_0, P_0, f, Q, N);
Y_s = genNonLinearMeasurementSequence(X_s, h, R);

% Get nonlinearKF result
[x_cs1_E, P_cs1_E, ~, ~] = nonLinearKalmanFilter(Y_s, x_0, P_0, f, Q, h, R, 'EKF');
[x_cs1_U, P_cs1_U, ~, ~] = nonLinearKalmanFilter(Y_s, x_0, P_0, f, Q, h, R, 'UKF');
[x_cs1_C, P_cs1_C, ~, ~] = nonLinearKalmanFilter(Y_s, x_0, P_0, f, Q, h, R, 'CKF');

% Change angle to position for the measurement
X_measure = zeros(2, length(Y_s));
for i = 1:length(Y_s)
    [x, y] = getPosFromMeasurement(Y_s(1,i), Y_s(2,i), [-200;100], [-200;-100]);
    X_measure(:, i) = [x; y];
end

% Create a figure with subplots
figure('Position', [300 300 1200 400]);

% Subplot for EKF
subplot(1, 3, 1);
scatter([-200, -200], [100, -100], 'MarkerFaceColor', [0 0.5 0.8]);
hold on;
plot(X_s(1,:), X_s(2,:), 'b');
plot(X_measure(1,:), X_measure(2,:), 'r--');
plot(x_cs1_E(1,:), x_cs1_E(2,:), 'cyan');
% Get 3sigma ellipse
for j = 1:5:size(x_cs1_E, 2)
     [se] = sigmaEllipse2D(x_cs1_E(1:2,j), P_cs1_E(1:2,1:2,j), 3, 100);
     plot(se(1,:), se(2,:), 'g');
end
grid on;
axis equal;
hold off;
xlabel('value of y1'); % Set x-axis label
ylabel('value of y2'); % Set y-axis label
legend('sensor position', 'true position', 'measurement position', 'estimated position', '$3\sigma$', 'interpreter', 'latex');
title('EKF');

% Subplot for UKF
subplot(1, 3, 2);
scatter([-200, -200], [100, -100], 'MarkerFaceColor', [0 0.5 0.8]);
hold on;
plot(X_s(1,:), X_s(2,:), 'b');
plot(X_measure(1,:), X_measure(2,:), 'r--');
plot(x_cs1_U(1,:), x_cs1_U(2,:), 'cyan');
% Get 3sigma ellipse
for j = 1:5:size(x_cs1_U ,2)
     [su] = sigmaEllipse2D(x_cs1_U(1:2,j), P_cs1_U(1:2,1:2,j), 3, 100);
     plot(su(1,:), su(2,:), 'g');
end
grid on;
axis equal;
hold off;
xlabel('value of y1'); % Set x-axis label
ylabel('value of y2'); % Set y-axis label
legend('sensor position', 'true position', 'measurement position', 'estimated position', '$3\sigma$', 'interpreter', 'latex');
title('UKF');

% Subplot for CKF
subplot(1, 3, 3);
scatter([-200, -200], [100, -100], 'MarkerFaceColor', [0 0.5 0.8]);
hold on;
plot(X_s(1,:), X_s(2,:), 'b');
plot(X_measure(1,:), X_measure(2,:), 'r--');
plot(x_cs1_C(1,:), x_cs1_C(2,:), 'cyan');
% Get 3sigma ellipse
for j = 1:5:size(x_cs1_C, 2)
     [sc] = sigmaEllipse2D(x_cs1_C(1:2,j), P_cs1_C(1:2,1:2,j), 3, 100);
     plot(sc(1,:), sc(2,:), 'g');
end
grid on;
axis equal;
hold off;
xlabel('value of y1'); % Set x-axis label
ylabel('value of y2'); % Set y-axis label
legend('sensor position', 'true position', 'measurement position', 'estimated position', '$3\sigma$', 'interpreter', 'latex');
title('CKF');


%% 3a
% Clear workspace
close all
clear all
clc

% Set random seed
rng(6)

% True track parameters
T = 0.1;            % Sampling period
K = 600;            % Length of time sequence
omega = zeros(1,K+1); % Allocate memory for turn rate
omega(150:450) = -pi/301/T; % Set turn rate
x0 = [0 0 20 0 omega(1)]'; % Initial state
X = zeros(length(x0),K+1); % Allocate memory for true track
X(:,1) = x0; % Set initial state

% Create true track
for i=2:K+1
    X(:,i) = coordinatedTurnMotion(X(:,i-1), T); % Simulate motion
    X(5,i) = omega(i); % Set turn rate
end

% Initial prior for Kalman filter
x0 = [0; 0; 0; 0; 0];
P0 = diag([10^2, 10^2, 10^2, ((5*pi)/180)^2, ((1*pi)/180)^2]);

% Sensor positions
s1 = [300; -100];
s2 = [300; -300];

% Measurement noise
sigma_phi_1 = pi/180;
sigma_phi_2 = pi/180;
R = diag([sigma_phi_1, sigma_phi_2]).^2;

% Generate measurements
h = @(x) dualBearingMeasurement(x,s1,s2);
Y = genNonLinearMeasurementSequence(X, h, R);
x(1,:) = ( s2(2)-s1(2) + tan(Y(1,:))*s1(1) - tan(Y(2,:))*s2(1) ) ./ ( tan(Y(1,:)) - tan(Y(2,:)) );
y(1,:) = s1(2) + tan(Y(1,:)) .* ( x(1,:) - s1(1) );
xpos = x(1,:);
ypos = y(1,:);
f = @(x) coordinatedTurnMotion(x, T);

% Kalman filter parameters
type = 'EKF'; % Filter type
sigma_v = 1; % Process noise
sigma_w = pi/180;

% Three different covariance matrices
Q_high = diag([0 0 (20*sigma_v)^2 0 (20*sigma_w)^2]).*T; % High process noise
Q_mid = diag([0 0 (sigma_v).^2 0 (sigma_w).^2]).*T; % Middle process noise
Q_low = diag([0 0 (0.001*sigma_v)^2 0 (0.001*sigma_w).^2]).*T; % Low process noise

% Perform Kalman filter for each covariance matrix
[xf_high,Pf_high,xp_high,Pp_high] = nonLinearKalmanFilter(Y,x0,P0,f,Q_high,h,R,type);
[xf_mid,Pf_mid,xp_mid,Pp_mid] = nonLinearKalmanFilter(Y,x0,P0,f,Q_mid,h,R,type);
[xf_low,Pf_low,xp_low,Pp_low] = nonLinearKalmanFilter(Y,x0,P0,f,Q_low,h,R,type);

% Plot true position
figure

% High Process Noise subplot
subplot(1,3,1)
grid on
hold on
plot(X(1,:),X(2,:),'k','linewidth',2) % True position
plot(xpos,ypos,':','linewidth',2,'color',[0 0.3 0.3]) % Measured position
plot([0, xf_high(1,:)], [0, xf_high(2,:)],'LineWidth',2) % Filtered position with high process noise
scatter(s1(1),s1(2),'k','filled','square')
scatter(s2(1),s2(2),'k','filled','square')
legend('True position','Measured position','Filtered position (High Process Noise)','Cam 1 pos','Cam 2 pos')
title(['High Process Noise (', '\sigma_v = ', num2str(sigma_v * 20), ', \sigma_w = ', num2str(sigma_w * 20), ')'])
xlabel('x pos'); ylabel('y pos')

% Middle Process Noise subplot
subplot(1,3,2)
grid on
hold on
plot(X(1,:),X(2,:),'k','linewidth',2) % True position
plot(xpos,ypos,':','linewidth',2,'color',[0 0.3 0.3]) % Measured position
plot([0, xf_mid(1,:)], [0, xf_mid(2,:)],'LineWidth',2) % Filtered position with middle process noise
scatter(s1(1),s1(2),'k','filled','square')
scatter(s2(1),s2(2),'k','filled','square')
legend('True position','Measured position','Filtered position (Middle Process Noise)','Cam 1 pos','Cam 2 pos')
title(['Middle Process Noise (', '\sigma_v = ', num2str(sigma_v), ', \sigma_w = ', num2str(sigma_w), ')'])
xlabel('x pos'); ylabel('y pos')

% Low Process Noise subplot
subplot(1,3,3)
grid on
hold on
plot(X(1,:),X(2,:),'k','linewidth',2) % True position
plot(xpos,ypos,':','linewidth',2,'color',[0 0.3 0.3]) % Measured position
plot([0, xf_low(1,:)], [0, xf_low(2,:)],'LineWidth',2) % Filtered position with low process noise
scatter(s1(1),s1(2),'k','filled','square')
scatter(s2(1),s2(2),'k','filled','square')
legend('True position','Measured position','Filtered position (Low Process Noise)','Cam 1 pos','Cam 2 pos')
title(['Low Process Noise (', '\sigma_v = ', num2str(sigma_v * 0.001), ', \sigma_w = ', num2str(sigma_w * 0.001), ')'])
xlabel('x pos'); ylabel('y pos')







%% 3b






%% 3c













% --------------------------------Functions---------------------------------------------
%% coordinatedTurnMotion
function [fx, Fx] = coordinatedTurnMotion(x, T)

posX = x(1);
posY = x(2);
velocity = x(3);
heading = x(4);
turnRate = x(5);

fx = [posX + (T*velocity*cos(heading));
      posY + T*velocity*sin(heading);
      velocity;
      heading + T*turnRate;
      turnRate];


if nargout > 1
Fx = [1     0   T * cos(x(4))     -T*x(3)*sin(x(4))   0;
      0     1   T * sin(x(4))     T*x(3)*cos(x(4))    0;
      0     0   1   0   0;
      0     0   0   1   T;
      0     0   0   0   1];

end
end

%% dualBearingMeasurement

function [hx, Hx] = dualBearingMeasurement(X, s1, s2)

x = X(1);
y = X(2);

delta1 = [x - s1(1); y - s1(2)];
delta2 = [x - s2(1); y - s2(2)];

hx  = [atan2(delta1(2),delta1(1)); atan2(delta2(2),delta2(1))];
Hx = zeros(2, length(X));
d1 = 1 / (delta1(1)^2 + delta1(2)^2) * [-delta1(2), delta1(1)];
d2 = 1 / (delta2(1)^2 + delta2(2)^2) * [-delta2(2), delta2(1)];

Hx(1,1:2) = d1;
Hx(2,1:2) = d2;
end

%% genNonLinearStateSequence

function X = genNonLinearStateSequence(x0, P0, f, Q, N)
    n = length(x0);
    X = zeros(n, N + 1);
        X(:, 1) = mvnrnd(x0, P0)';
    
    for i = 2:N+1
        v = mvnrnd(zeros(1, n), Q)';
        
        [fx, Fx] = f(X(:, i-1));
        X(:, i) = fx + v;
    end
end

%% genNonLinearMeasurementSequence

function Y = genNonLinearMeasurementSequence(X, h, R)
    [n, N] = size(X);
    m = size(R, 1);
    
    Y = zeros(m, N - 1);
    
    for i = 2:N
        w = mvnrnd(zeros(1, m), R)';
        [hx, ~] = h(X(:, i));
        Y(:, i-1) = hx + w;
    end
end

%% sigmaPoints

function [SP,W] = sigmaPoints(x, P, type)

    n = length(x);

    switch type        
        case 'UKF'
 
             SP = zeros(n,2*n+1);
             W = zeros(1,2*n+1);
             
             SP(:,1) = x;
             W(1) = 1 - (n/3);
             
             Pchol = chol(P,'lower');
             for i = 2:(n+1)
                SP(:,i) = x + (sqrt(n / (1 - W(1)))*Pchol(:,i-1));
                SP(:,i+n) = x - (sqrt(n / (1 - W(1)))*Pchol(:,i-1));
             end
             W(2:end) = (1 - W(1)) / (2*n);
       
        case 'CKF'

             SP = zeros(n,2*n);
             W = zeros(1,2*n);
             Pchol = chol(P,'lower');
             for i = 1:n
                SP(:,i)   = x + sqrt(n)*Pchol(:,i);
                SP(:,i+n) = x - sqrt(n)*Pchol(:,i);
             end
            
             W(:) = 1/(2*n);
        otherwise
            error('Incorrect type of sigma point')
    end

end

%% Non-linear Kalman prediction




%% nonLinKFpredictions
function [x, P] = nonLinKFprediction(x, P, f, Q, type)


    switch type
        case 'EKF'
            
            % Your EKF code here
            [fx, Fx] = f(x);
            x = fx;
            P = Fx * P * Fx' + Q;
            
        case 'UKF'
    
            % Your UKF code here
            [X,W] = sigmaPoints(x, P, 'UKF');
            n = length(x);
            xk = 0;
            Pk = 0;
            for i=1:(2*n+1)
                xk = xk + f(X(:,i))*W(:,i);
            end
           
            Ptemp = 0;
            for i=1:(2*n+1)
                Ptemp = Ptemp + ( ( f(X(:,i)) - xk ) * ( ( f(X(:,i)) - xk )' )*W(:,i));
            end
            Pk = Q + Ptemp;

            x = xk;
            P = Pk;

            % Make sure the covariance matrix is semi-definite
            if min(eig(P))<=0
                [v,e] = eig(P, 'vector');
                e(e<0) = 1e-4;
                P = v*diag(e)/v;
            end
                
        case 'CKF'
            
            % Your CKF code here
            [X,W] = sigmaPoints(x, P, 'CKF');
            n = length(x);
            xk = 0;
            Pk = 0;
            for i=1:(2*n)
                xk = xk + f(X(:,i))*W(:,i);
            end
           
            Ptemp = 0;
            for i=1:(2*n)
                Ptemp = Ptemp + ( ( f(X(:,i)) - xk ) * ( ( f(X(:,i)) - xk )' )*W(:,i));
            end
            Pk = Q + Ptemp;

            x = xk;
            P = Pk;
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end

end


%%  Nonlinear Kalman update

function [x, P] = nonLinKFupdate(x, P, y, h, R, type)



n = length(x);

    switch type
        case 'EKF'
            [hx,Hx]=h(x);
            S = Hx*P*Hx' + R;
            K = P*Hx'*inv(S);
            x = x + K*(y - hx);
            P = P - K*S*K';    
        case 'UKF'

            [SP,W] = sigmaPoints(x, P, 'UKF');
    
            y_hat = 0;
            for i=1:(2*n)+1
                [hx,]=h(SP(:,i));
                y_hat = y_hat + hx*W(i);
            end
            
            P_xy = 0;
            for i=1:(2*n)+1
                [hx,]=h(SP(:,i));
                P_xy = P_xy + ((SP(:,i) - x) * (hx - y_hat).' * W(i));
            end
            
            S = R;
            for i=1:(2*n)+1
                [hx,]=h(SP(:,i));
                S = S + (hx - y_hat)*(hx - y_hat).'*W(i);
            end
            x = x + P_xy*inv(S)*(y - y_hat);
            P = P - P_xy*inv(S)*P_xy';
            
            if min(eig(P))<=0
                [v,e] = eig(P, 'vector');
                e(e<0) = 1e-4;
                P = v*diag(e)/v;
            end
         case 'CKF'
    
            [SP,W] = sigmaPoints(x, P, 'CKF');
            
            y_hat = 0;
            for i=1:(2*n)
                [hx,]=h(SP(:,i));
                y_hat = y_hat + hx*W(i);
            end
            
            P_xy = 0;
            for i=1:(2*n)
                [hx,]=h(SP(:,i));
                P_xy = P_xy + ((SP(:,i) - x) * (hx - y_hat).' * W(i));
            end
            
            S = R;
            for i=1:(2*n)
                [hx,]=h(SP(:,i));
                S = S + (hx - y_hat)*(hx - y_hat).'*W(i);
            end
            x = x + P_xy*inv(S)*(y - y_hat);
            P = P - P_xy*inv(S)*P_xy';
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end
end
%% Non-linear Kalman filter
function [xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, type)

N = size(Y,2);
n = length(x_0);
xf = zeros(n,N);
Pf = zeros(n,n,N);
xp = zeros(n,N);
Pp = zeros(n,n,N);

for i =1:N
    
    [x_pred, P_pred] = nonLinKFprediction(x_0, P_0, f, Q, type);
    xp(:,i) = x_pred;
    Pp(:,:,i) = P_pred;
    
    [x_f, P_f] = nonLinKFupdate(x_pred, P_pred, Y(:,i), h, R, type);
    xf(:,i) = x_f;
    Pf(:,:,i) = P_f;
    
    x_0 = xf(:,i);
    P_0 = Pf(:,:,i);

end
end 

%% 
function [mu_y, Sigma_y, y_s] = approxGaussianTransform(mu_x, Sigma_x, f, N)

if nargin < 4
    N = 5000;
end
x_s = mvnrnd(mu_x, Sigma_x, N)';
y_s = f(x_s);
mu_y = mean(y_s,2);
Sigma_y = cov(y_s');

end

%% 
function [ xy ] = sigmaEllipse2D( mu, Sigma, level, npoints )

if nargin < 3
    level = 3;
end
if nargin < 4
    npoints = 32;
end

phi = linspace(0,2*pi,npoints);
z = level*[cos(phi); sin(phi)];
xy = mu + sqrtm(Sigma)*z;

end

%% 
function [mean_y, P_y] = analyticalapprox(x, P, h, R, type)
%ANALYTICALDENSITY Summary of this function goes here
%   Detailed explanation goes here


    % estimate transformed mean and covariance
    if all(type == 'UKF') || all(type == 'CKF')
    
        % Determine som sigma points around x
        [SP,W] = sigmaPoints(x,P,type);
        
        % Propagate sigma points through non-linear function
        [mean_y, P_y]  = propagateSP(h, R, SP, W);
    
    elseif all(type == 'EKF')
        [hx, dhx] = h(x);
        mean_y = hx;
        P_y = (dhx * P * dhx') + R;
    end



function [x, P] = propagateSP(f, R, SP, W)
    n = size(SP,1);
    x = zeros(n,1);
    for i=1:numel(W)
        x = x + f(SP(:,i)) * W(i);
    end   
    P = R; %zeros(n,n);
    for i=1:numel(W)
        P = P + (f(SP(:,i))-x)*(f(SP(:,i))-x).' * W(i);
    end

end
end

%% 
function [x, y] = getPosFromMeasurement(y1, y2, s1, s2)
x = (s2(2)-s1(2)+tan(y1)*s1(1)-tan(y2)*s2(1))/(tan(y1)-tan(y2));
y = s1(2)+tan(y1)*(x-s1(1));
end

