


%% q1
close all
clear
clc


% Parameters
T = 0.1;          % Sampling time
K = 600;          % Length of time sequence

% True track initialization
omega = zeros(1, K + 1);
omega(150:450) = -pi / 301 / T;  % Turn rate

% Initial state
x0 = [0; 0; 20; 0; omega(1)]; 

% Memory allocation
X = zeros(length(x0), K + 1);
X(:, 1) = x0;

% True track generation
for i = 2:K+1
    X(:, i) = coordinatedTurnMotion(X(:, i - 1), T);
    X(5, i) = omega(i);  % Set turn-rate
end

% Time vector
Tvec = 0:T:(length(X) - 1) * T;

% Initial prior
x0_prior = [0; 0; 0; 0; 0];
P0_prior = diag([10^2, 10^2, 10^2, ((5*pi)/180)^2, ((1*pi)/180)^2]);

% Sensor positions
s1 = [300; -100];
s2 = [300; -300];
S = [s1, s2];

% Measurement noise
sigma_phi_1 = pi/180;
sigma_phi_2 = pi/180;
R = diag([sigma_phi_1^2, sigma_phi_2^2]) * T;

% Process noise covariance
sigma_v = 1;
sigma_w = pi/180;
Q = diag([0, 0, sigma_v^2, 0, sigma_w^2]) * T;

% Generate measurements
h = @(x, ~) dualBearingMeasurement(x, s1, s2);
Y = genNonLinearMeasurementSequence(X, h, R, S);
[xpos, ypos] = getPosFromMeasurement(Y, s1, s2);

% Dynamics model
proc_f = @(x, ~) coordinatedTurnMotion(x, T);
SigmaPoints = @sigmaPoints;

% Dummy variable for sensor pos
S = zeros(2, length(Y));

% Filter the position with non-linear Kalman filter
type = 'CKF';
[xs, Ps, xf, Pf, xp, Pp] = nonLinRTSsmoother(Y, x0_prior, P0_prior, proc_f, T, Q, S, h, R, SigmaPoints, type);

% Plotting
plotFilterSmoother(X, xs, Ps, xf, Pf, xp, Pp, xpos, ypos, s1, s2, type, Tvec);

% Modify a measurement at k=300
Y(:, 300) = Y(:, 300) + rand();
[xpos, ypos] = getPosFromMeasurement(Y, s1, s2);

% Filter the position with non-linear Kalman filter
[xs, Ps, xf, Pf, xp, Pp] = nonLinRTSsmoother(Y, x0_prior, P0_prior, proc_f, T, Q, S, h, R, SigmaPoints, type);

% Plot everything
plotFilterSmoother(X, xs, Ps, xf, Pf, xp, Pp, xpos, ypos, s1, s2, type, Tvec);




%%

% Clean workspace
close all
clear
clc

% Set random seed
rng(5)

% Define process and measurement noise
proc_Q = 1.5;
meas_R = 3;

% Initial prior and covariance
x0 = 2;
P0 = 8;

% Timesteps
K = 30;

% Define dynamics and measurement models
proc_f = @(x) x;
meas_h = @(x) x;

% Generate state and measurement sequences
X = genLinearStateSequence(x0, P0, proc_f, proc_Q, K);
Y = genLinearMeasurementSequence(X, meas_h, meas_R);

% Incorrect prior (Uncomment for 2b)
% x0 = -20;
% P0 = 2;

% Kalman Filter
[xf_KF, Pf_KF] = linearKalmanFilter(Y, x0, P0, 1, proc_Q, 1, meas_R);

% Define N values for particle filter
N_values = [10, 100, 1000, 10000];

% Initialize arrays to store MSEs
err_KF_values = zeros(size(N_values));
err_PF_values = zeros(size(N_values));
err_PFresam_values = zeros(size(N_values));

% Loop over N values
for i = 1:numel(N_values)
    N = N_values(i);
    
    % Particle Filter
    [xf_PF, Pf_PF, ~, ~] = pfFilter(x0, P0, Y, proc_f, proc_Q, meas_h, meas_R, N, false);
    [xf_PFresam, Pf_PFresam, ~, ~] = pfFilter(x0, P0, Y, proc_f, proc_Q, meas_h, meas_R, N, true);

    % Calculate MSEs
    err_KF_values(i) = immse(X(2:end), xf_KF);
    err_PF_values(i) = immse(X(2:end), xf_PF);
    err_PFresam_values(i) = immse(X(2:end), xf_PFresam);
end

% Plotting
plotParticleFilter(X, Y, K, xf_KF, Pf_KF, xf_PF, Pf_PF, xf_PFresam, Pf_PFresam);
time_instances = [1, 15, 30];
plotPosteriorApproximation(xf_KF, Pf_KF, xf_PF, Pf_PF, xf_PFresam, Pf_PFresam, time_instances);
plotParticleFilterErrorbar(X, Y, K, xf_KF, Pf_KF, xf_PF, Pf_PF, xf_PFresam, Pf_PFresam);
N_table = [N_values', err_KF_values', err_PF_values', err_PFresam_values'];
disp(array2table(N_table, 'VariableNames', {'N', 'MSE_KF', 'MSE_PF', 'MSE_PFresam'}));

% Particle Trajectory Plot
N = 100; % Choose N for the plot
bResampl = true;
[~, ~, Xp_PF, Wp_PF] = pfFilter(x0, P0, Y, proc_f, proc_Q, meas_h, meas_R, N, bResampl);
plotParticleTrajectory(Xp_PF, Wp_PF, K, N, X, xf_PFresam, Pf_PFresam, bResampl);

%%
function [xs, Ps, xf, Pf, xp, Pp] = nonLinRTSsmoother(Y, x_0, P_0, f, T, Q, S, h, R, sigmaPoints, type)

% [xPred, PPred] = nonLinKFprediction(x_0, P_0, f, T, Q, sigmaPoints, type);
% [xf, Pf] = nonLinKFupdate(xPred, PPred, Y, S, h, R, sigmaPoints, type);
    switch type
        case 'EKF'
            n = size(x_0,1);
            N = size(Y,2);
            xp = zeros(n,N);
            Pp = zeros(n,n,N);
            xf = zeros(n,N);
            Pf = zeros(n,n,N);
            xs = zeros(n,N);
            Ps = zeros(n,n,N);
            for k=1:N
                if k==1
                    [xp(:,k), Pp(:,:,k)] = nonLinKFprediction(x_0, P_0, f, T, Q, sigmaPoints, 'EKF');
                else
                    [xp(:,k), Pp(:,:,k)] = nonLinKFprediction(xf(:,k-1), Pf(:,:,k-1), f, T, Q, sigmaPoints, 'EKF');
                end
                [xf(:,k), Pf(:,:,k)] = nonLinKFupdate(xp(:,k), Pp(:,:,k), Y(:,k), S(:,k), h, R, sigmaPoints, 'EKF');
            end
            xs(:,N) = xf(:,N);
            Ps(:,:,N) = Pf(:,:,N); 
            for k=N-1:-1:1
                [xs(:,k),Ps(:,:,k)] = nonLinRTSSupdate(xs(:,k+1), Ps(:,k+1), xf(:,k), Pf(:,:,k), xp(:,k+1), Pp(:,:,k+1), f, T, sigmaPoints, 'EKF');
            end
        case 'UKF'
            n = size(x_0,1);
            N = size(Y,2);
            xp = zeros(n,N);
            Pp = zeros(n,n,N);
            xf = zeros(n,N);
            Pf = zeros(n,n,N);
            xs = zeros(n,N);
            Ps = zeros(n,n,N);
            for k=1:N
                if k==1
                    [xp(:,k), Pp(:,:,k)] = nonLinKFprediction(x_0, P_0, f, T, Q, sigmaPoints, 'UKF');
                else
                    [xp(:,k), Pp(:,:,k)] = nonLinKFprediction(xf(:,k-1), Pf(:,:,k-1), f, T, Q, sigmaPoints, 'UKF');
                end
                [xf(:,k), Pf(:,:,k)] = nonLinKFupdate(xp(:,k), Pp(:,:,k), Y(:,k), S(:,k), h, R, sigmaPoints, 'UKF');
            end
            xs(:,N) = xf(:,N);
            Ps(:,:,N) = Pf(:,:,N); 
            for k=N-1:-1:1
                [xs(:,k),Ps(:,:,k)] = nonLinRTSSupdate(xs(:,k+1), Ps(:,k+1), xf(:,k), Pf(:,:,k), xp(:,k+1), Pp(:,:,k+1), f, T, sigmaPoints, 'EKF');
            end
            
        case 'CKF'
            n = size(x_0,1);
            N = size(Y,2);
            xp = zeros(n,N);
            Pp = zeros(n,n,N);
            xf = zeros(n,N);
            Pf = zeros(n,n,N);
            xs = zeros(n,N);
            Ps = zeros(n,n,N);
            for k=1:N
                if k==1
                    [xp(:,k), Pp(:,:,k)] = nonLinKFprediction(x_0, P_0, f, T, Q, sigmaPoints, 'CKF');
                else
                    [xp(:,k), Pp(:,:,k)] = nonLinKFprediction(xf(:,k-1), Pf(:,:,k-1), f, T, Q, sigmaPoints, 'CKF');
                end
                [xf(:,k), Pf(:,:,k)] = nonLinKFupdate(xp(:,k), Pp(:,:,k), Y(:,k), S(:,k), h, R, sigmaPoints, 'CKF');
            end
            xs(:,N) = xf(:,N);
            Ps(:,:,N) = Pf(:,:,N); 
            for k=N-1:-1:1
                [xs(:,k),Ps(:,:,k)] = nonLinRTSSupdate(xs(:,k+1), Ps(:,k+1), xf(:,k), Pf(:,:,k), xp(:,k+1), Pp(:,:,k+1), f, T, sigmaPoints, 'EKF');
            end
            
        otherwise 
            error('Incorrect type of sigma point')
    end
end
%%
function [xs, Ps] = nonLinRTSSupdate(xs_kplus1, Ps_kplus1, xf_k, Pf_k, xp_kplus1, Pp_kplus1, f, T, sigmaPoints, type)
    % Your code here! Copy from previous task!
    switch type        
        case 'EKF'
            [fxf_k, Fxf_k] = f(xf_k,T);
            Gk = Pf_k * Fxf_k' * inv(Pp_kplus1);
            xs  = xf_k + Gk*(xs_kplus1 - fxf_k);
            Ps = Pf_k - Gk*(Pp_kplus1 - Ps_kplus1)*Gk';            
    
        case 'UKF'
           [X,W] = sigmaPoints(xf_k, Pf_k, 'UKF');
           n = length(xf_k);
           %xhat_kplus1_k = 0;
           %for i=1:(2*n+1)
           %    xhat_kplus1_k = xhat_kplus1_k + f(X(:,i),T)*W(:,i);
           %end
           
           Pk_kplus1_k = 0;
           Ptemp = 0;
           for i=1:(2*n+1)
               Ptemp = Ptemp + ((X(:,i) - xf_k) * ( ( f(X(:,i),T) - xp_kplus1 )' )*W(:,i));
           end
           Pk_kplus1_k = Ptemp;

           Gk = Pk_kplus1_k*inv(Pp_kplus1);
           xs = xf_k + Gk*(xs_kplus1 - xp_kplus1);
           Ps = Pf_k - Gk*(Pp_kplus1 - Ps_kplus1)*Gk';

        case 'CKF'
           [X,W] = sigmaPoints(xf_k, Pf_k, 'CKF');
           n = length(xf_k);
           
           Pk_kplus1_k = 0;
           Ptemp = 0;
           for i=1:(2*n)
               Ptemp = Ptemp + ((X(:,i) - xf_k) * ( ( f(X(:,i),T) - xp_kplus1 )' )*W(:,i));
           end
           Pk_kplus1_k = Ptemp;

           Gk = Pk_kplus1_k*inv(Pp_kplus1);
           xs = xf_k + Gk*(xs_kplus1 - xp_kplus1);
           Ps = Pf_k - Gk*(Pp_kplus1 - Ps_kplus1)*Gk';

        otherwise
            error('Incorrect type of sigma point')
    end
end

%%
function [SP,W] = sigmaPoints(x, P, type)

    switch type        
        case 'UKF'

            % Dimension of state
            n = length(x);

            % Allocate memory
            SP = zeros(n,2*n+1);

            % Weights
            W = [1-n/3 repmat(1/6,[1 2*n])];

            % Matrix square root
            sqrtP = sqrtm(P);

            % Compute sigma points
            SP(:,1) = x;
            for i = 1:n
                SP(:,i+1) = x + sqrt(1/2/W(i+1))*sqrtP(:,i);
                SP(:,i+1+n) = x - sqrt(1/2/W(i+1+n))*sqrtP(:,i);
            end

        case 'CKF'

            % Dimension of state
            n = length(x);

            % Allocate memory
            SP = zeros(n,2*n);

            % Weights
            W = repmat(1/2/n,[1 2*n]);

            % Matrix square root
            sqrtP = sqrtm(P);

            % Compute sigma points
            for i = 1:n
                SP(:,i) = x + sqrt(n)*sqrtP(:,i);
                SP(:,i+n) = x - sqrt(n)*sqrtP(:,i);
            end

        otherwise
            error('Incorrect type of sigma point')
    end
end
%%
function [h, H] = rangeBearingMeasurements(x, s)

    rng = norm(x(1:2)-s);
    ber = atan2(x(2)-s(2),x(1)-s(1));
    h = [rng;ber];
    H = [
        (x(1)-s(1))/rng      (x(2)-s(2))/rng     0 0 0;
        -(x(2)-s(2))/(rng^2) (x(1)-s(1))/(rng^2) 0 0 0
        ];

end
%%
function [f, F] = coordinatedTurnMotion(x, T)
    v = x(3);
    phi = x(4);
    omega = x(5);

    f = x + [
        T*v*cos(phi);
        T*v*sin(phi);
        0;
        T*omega;
        0];

    F = [
        1 0 T*cos(phi) -T*v*sin(phi) 0;
        0 1 T*sin(phi) T*v*cos(phi)  0;
        0 0 1          0             0;
        0 0 0          1             T;
        0 0 0          0             1
        ];
end
%%
function X = genNonLinearStateSequence(x_0, P_0, f, T, Q, N)

    n = length(x_0);

    X = zeros(n, N);

    X(:,1) = mvnrnd(x_0', P_0)';

    for k = 2:N+1

        q = mvnrnd(zeros(1,n), Q)';

        [fX, ~] = f(X(:,k-1),T);
        X(:,k) = fX + q;

    end

end
%%
function Y = genNonLinearMeasurementSequence(X, h, R, s)
% X [n x N+1] State vector sequence
    % Extract the number of states and measurements
    [n, N] = size(X);
    m = size(R, 1);
    
    % Initialize the measurement sequence
    Y = zeros(m, N-1);%X size is N+1
    
    % Generate the measurement sequence
    for k = 1:N-1
        % Propagate the state through the measurement model
        [hx, Hx] = h(X(:, k+1),s);
        
        % Draw a random sample from the measurement noise distribution
        v_k = mvnrnd(zeros(m, 1), R)';
        
        % Generate the measurement by adding noise to the predicted measurement
        Y(:, k) = hx + v_k;
    end
end

%%
function [x, P] = nonLinKFprediction(x, P, f, T, Q, sigmaPoints, type)

    switch type
        case 'EKF'

            % Evaluate motion model
            [fx, Fx] = f(x,T);
            % State prediction
            x = fx;
            % Covariance prediciton
            P = Fx*P*Fx' + Q;
            % Make sure P is symmetric
            P = 0.5*(P + P');

        case 'UKF'

            % Predict
            [x, P] = predictMeanAndCovWithSigmaPoints(x, P, f, T, Q, sigmaPoints, type);

            if min(eig(P))<=0
                [v,e] = eig(P);
                emin = 1e-3;
                e = diag(max(diag(e),emin));
                P = v*e*v';
            end

        case 'CKF'

            % Predict
            [x, P] = predictMeanAndCovWithSigmaPoints(x, P, f, T, Q, sigmaPoints, type);

        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end
end

%%
function [x, P] = nonLinKFupdate(x, P, y, s, h, R, sigmaPoints, type)
%NONLINKFUPDATE calculates mean and covariance of predicted state
%   density using a non-linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   y           [m x 1] measurement vector
%   s           [2 x 1] sensor position vector
%   h           Measurement model function handle
%   R           [n x n] Measurement noise covariance
%   sigmaPoints Handle to function that generates sigma points.
%   type        String that specifies the type of non-linear filter
%
%Output:
%   x           [n x 1] updated state mean
%   P           [n x n] updated state covariance
%


switch type
    case 'EKF'
        
        % Evaluate measurement model
        [hx, Hx] = h(x,s);
        
        % Innovation covariance
        S = Hx*P*Hx' + R;
        % Kalman gain
        K = (P*Hx')/S;
        
        % State update
        x = x + K*(y - hx);
        % Covariance update
        P = P - K*S*K';
        
        % Make sure P is symmetric
        P = 0.5*(P + P');
        
    case 'UKF'

        % Update mean and covariance
        [x, P] = updateMeanAndCovWithSigmaPoints(x, P, y, s, h, R, sigmaPoints, type);
        
        if min(eig(P))<=0
            [v,e] = eig(P);
            emin = 1e-3;
            e = diag(max(diag(e),emin));
            P = v*e*v';
        end
        
    case 'CKF'

        % Update mean and covariance
        [x, P] = updateMeanAndCovWithSigmaPoints(x, P, y, s, h, R, sigmaPoints, type);
        
    otherwise
        error('Incorrect type of non-linear Kalman filter')
end
end
%%
function [x, P] = predictMeanAndCovWithSigmaPoints(x, P, f, T, Q, sigmaPoints, type)
    % Compute sigma points
    [SP,W] = sigmaPoints(x, P, type);

    % Dimension of state and number of sigma points
    [n, N] = size(SP);

    % Allocate memory
    fSP = zeros(n,N);

    % Predict sigma points
    for i = 1:N
        [fSP(:,i),~] = f(SP(:,i),T);
    end

    % Compute the predicted mean
    x = sum(fSP.*repmat(W,[n, 1]),2);

    % Compute predicted covariance
    P = Q;
    for i = 1:N
        P = P + W(i)*(fSP(:,i)-x)*(fSP(:,i)-x)';
    end

    % Make sure P is symmetric
    P = 0.5*(P + P');

end


%%
function [x, P] = updateMeanAndCovWithSigmaPoints(x, P, y, s, h, R, sigmaPoints, type)

    % Compute sigma points
    [SP,W] = sigmaPoints(x, P, type);

    % Dimension of measurement
    m = size(R,1);

    % Dimension of state and number of sigma points
    [n, N] = size(SP);

    % Predicted measurement
    yhat = zeros(m,1);
    hSP = zeros(m,N);
    for i = 1:N
        [hSP(:,i),~] = h(SP(:,i),s);
        yhat = yhat + W(i)*hSP(:,i);
    end

    % Cross covariance and innovation covariance
    Pxy = zeros(n,m);
    S = R;
    for i=1:N
        Pxy = Pxy + W(i)*(SP(:,i)-x)*(hSP(:,i)-yhat)';
        S = S + W(i)*(hSP(:,i)-yhat)*(hSP(:,i)-yhat)';
    end

    % Ensure symmetry
    S = 0.5*(S+S');

    % Updated mean
    x = x+Pxy*(S\(y-yhat));
    P = P - Pxy*(S\(Pxy'));

    % Ensure symmetry
    P = 0.5*(P+P');

end


%%
function [xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Y, proc_f, proc_Q, meas_h, meas_R, N, bResample, plotFunc)
K = size(Y,2);
n = size(x_0,1);
Xp = zeros(n,N,K);
Wp = zeros(N,K);
for k=1:K
    if k == 1
        X_kmin1 = mvnrnd(x_0,P_0,N);
        W_kmin1 = 1/N*ones(1,N);
        [Xp(:,:,k),Wp(:,k)] = pfFilterStep(X_kmin1', W_kmin1, Y(:,k), proc_f, proc_Q, meas_h, meas_R);
    else
        X_kmin1 = Xp(:,:,k-1);
        W_kmin1 = Wp(:,k-1)';
        [Xp(:,:,k),Wp(:,k)] = pfFilterStep(X_kmin1, W_kmin1, Y(:,k), proc_f, proc_Q, meas_h, meas_R);
    end
    if bResample == 1
        [Xp(:,:,k),Wp(:,k),~] = resampl(Xp(:,:,k),Wp(:,k)');
    end
xfp(:,k) = sum( Xp(:,:,k).*Wp(:,k)' , 2);
Pfp(:,:,k) = Wp(:,k)' .* ( xfp(:,k) - Xp(:,:,k) )*(( xfp(:,k) - Xp(:,:,k) )');
end
end
%% 
function [Xr, Wr, j] = resampl(X, W)
    n = size(X,1);
    N = size(W,2);
    Wr = zeros(1,N);
    Wr(1,1:end) = 1/N;
    
    cumW = cumsum(W);
    
    Xr = zeros(n,N);
    j=zeros(1,N);
    for i = 1:N
        rand_num = rand();
        [~, idx] = max(cumW >= rand_num);
        j(:,i) = idx;
        Xr(:,i) = X(:,idx);
    end
end
%% 
    function [X_k, W_k] = pfFilterStep(X_kmin1, W_kmin1, yk, proc_f, proc_Q, meas_h, meas_R)
        X_k = mvnrnd(proc_f(X_kmin1)', proc_Q)';
        Wy = mvnpdf(yk', meas_h(X_k)', meas_R)';
        
        W_k = W_kmin1 .* Wy;
        W_k = W_k / sum(W_k);
    end
%%
function [hx, Hx] = dualBearingMeasurement(X, s1, s2)
%DUOBEARINGMEASUREMENT calculates the bearings from two sensors, located in 
%s1 and s2, to the position given by the state vector x. Also returns the
%Jacobian of the model at x.
%
%Input:
%   x           [n x 1] State vector, the two first element are 2D position
%   s1          [2 x 1] Sensor position (2D) for sensor 1
%   s2          [2 x 1] Sensor position (2D) for sensor 2
%
%Output:
%   hx          [2 x 1] measurement vector
%   Hx          [2 x n] measurement model Jacobian
%
% NOTE: the measurement model assumes that in the state vector x, the first
% two states are X-position and Y-position.

% Your code here
x = X(1);
y = X(2);

delta1 = [x - s1(1); y - s1(2)];
delta2 = [x - s2(1); y - s2(2)];

hx  = [atan2(delta1(2),delta1(1)); atan2(delta2(2),delta2(1))];

% Jacobian
Hx = zeros(2, length(X));

% Derivatives of atan2
d1 = 1 / (delta1(1)^2 + delta1(2)^2) * [-delta1(2), delta1(1)];
d2 = 1 / (delta2(1)^2 + delta2(2)^2) * [-delta2(2), delta2(1)];


% Filling in the Jacobian matrix
Hx(1,1:2) = d1;
Hx(2,1:2) = d2;
end

%%
function [X,T,Tvec] = generateTrueTrack()
% True track
% Sampling period
T = 0.1;

% Length of time sequence
K = 600;

% Allocate memory
omega = zeros(1,K+1);

% Turn rate
omega(150:450) = -pi/301/T;

% Initial state
x0 = [0 0 20 0 omega(1)]';

% Allocate memory
X = zeros(length(x0),K+1);
X(:,1) = x0;

% Create true track
for i=2:K+1
    % Simulate
    X(:,i) = coordinatedTurnMotion(X(:,i-1), T);
    % Set turn−rate
    X(5,i) = omega(i);
end

Tvec = 0:T:(length(X)-1)*T;

end


%%
function [x, y] = getPosFromMeasurement(Y, s1, s2)
%GETPOSFROMMEASUREMENT computes the intersection point
%(transformed 2D measurement in Cartesian coordinate
%system) given two sensor locations and two bearing
%measurements, one from each sensor.
%INPUT: y1: bearing measurement from sensor 1
% y2: bearing measurement from sensor 2
% s1: location of sensor 1 in 2D Cartesian
% s2: location of sensor 2 in 2D Cartesian
%OUTPUT: x: coordinate of intersection point on x axis
% y: coordinate of intersection point on y axis
%This problem can be formulated as solving a set of two
%linear equations with two unknowns. Specifically, one
%would like to obtain (x,y) by solving
%(y−s1(2))=(x−s1(1))tan(y1) and (y−s2(2))=(x−s2(1))tan(y2).
x(1,:) = ( s2(2)-s1(2) + tan(Y(1,:))*s1(1) - tan(Y(2,:))*s2(1) ) ./ ( tan(Y(1,:)) - tan(Y(2,:)) );
y(1,:) = s1(2) + tan(Y(1,:)) .* ( x(1,:) - s1(1) );
end

%%
function plotFilterSmoother(X,xs, Ps, xf, Pf, xp, Pp, xpos, ypos, s1, s2, type, Tvec)
    %PLOTFILTERSMOOTHER Summary of this function goes here
    %   Helper function for plotting of the filter coordinated turn motion
    %   model


    figure()
  grid on; hold on
    plot(X(1,:), X(2,:), 'b', 'LineWidth', 2, 'MarkerEdgeColor', 'b')
    
    plot(xpos, ypos, ':', 'color', '#FF5733', 'LineWidth', 2)
    plot(xf(1,:), xf(2,:), 'g', 'LineWidth', 2)
    plot(xs(1,:), xs(2,:), 'm', 'LineWidth', 2)
    
    % Plot camera position
    scatter(s1(1), s1(2), 'r', 'filled', 'square')
    scatter(s2(1), s2(2), 'c', 'filled', 'square')

    
    % Plot 3sigma at every fifth instant
    for i = 5:5:length(X)-1
    
        P_i = Pf(1:2,1:2,i);
    
        %Generate 2D elipse
        [xy] = sigmaEllipse2D( [xf(1,i); xf(2,i)], P_i, 3, 1000);
    
        % Plot sigma curve
        %scatter(xy(1,:), xy(2,:), 2,'k','filled')
        plot(xy(1,:),xy(2,:),'k')
    
    end

    % Plot 3sigma at every fifth instant
    for i = 5:5:length(X)-1
    
        P_i = Ps(1:2,1:2,i);
    
        %Generate 2D elipse
        [xy] = sigmaEllipse2D( [xs(1,i); xs(2,i)], P_i, 3, 1000);
   
    
        % Plot sigma curve
        plot(xy(1,:), xy(2,:),'color','#A2142F')
    
    end
    
    % Set title/legend etc
    legend('True position','Measured position','Filtered position','Smoothed position','Cam 1 pos','Cam 2 pos','3\sigma-contour filter','3\sigma-contour smoother')
    title('Corrdinated turn motion model filtered with ',type)
    xlabel('x pos'); ylabel('y pos')
    
    
    figure()
    subplot(1,3,1)
    grid on; hold on
    plot(Tvec,X(3,:),'k','LineWidth',2)
    plot(Tvec(2:end),xf(3,:),'Color','#EDB120')
    plot(Tvec(2:end),xs(3,:),'Color','#7E2F8E','LineWidth',2)
    ylabel('Velocity [m/s]'); xlabel('time [s]')
    legend('True state trajectory','Filtered state trajectory','Smoothed state trajectory')
    
    subplot(1,3,2)
    grid on; hold on
    plot(Tvec,X(4,:),'k','LineWidth',2)
    plot(Tvec(2:end),xf(4,:),'Color','#EDB120')
    plot(Tvec(2:end),xs(4,:),'Color','#7E2F8E','LineWidth',2)
    ylabel('Turn (heading) [rad]'); xlabel('time [s]')
    legend('True state trajectory','Filtered state trajectory','Smoothed state trajectory')
    
    subplot(1,3,3)
    grid on; hold on
    plot(Tvec,X(5,:),'k','LineWidth',2)
    plot(Tvec(2:end),xf(5,:),'Color','#EDB120')
    plot(Tvec(2:end),xs(5,:),'Color','#7E2F8E','LineWidth',2)
    ylabel('Turn rate [rad/s]'); xlabel('time [s]')
    legend('True state trajectory','Filtered state trajectory','Smoothed state trajectory')

end



%%

function [ xy ] = sigmaEllipse2D( mu, Sigma, level, npoints )
%SIGMAELLIPSE2D generates x,y-points which lie on the ellipse describing
% a sigma level in the Gaussian density defined by mean and covariance.
%
%Input:
%   MU          [2 x 1] Mean of the Gaussian density
%   SIGMA       [2 x 2] Covariance matrix of the Gaussian density
%   LEVEL       Which sigma level curve to plot. Can take any positive value, 
%               but common choices are 1, 2 or 3. Default = 3.
%   NPOINTS     Number of points on the ellipse to generate. Default = 32.
%
%Output:
%   XY          [2 x npoints] matrix. First row holds x-coordinates, second
%               row holds the y-coordinates. First and last columns should 
%               be the same point, to create a closed curve.


%Setting default values, in case only mu and Sigma are specified.
if nargin < 3
    level = 3;
end
if nargin < 4
    npoints = 32;
end

%Your code here
% Edited by: Nicholas Granlund

% Define npoints from 0 to 2pi
phi = linspace(0,2*pi,npoints);

% points on the circle given by sigma level curve 'level'
z = level*[cos(phi); sin(phi)];

% Determine the points according to equation (2)
xy = mu + sqrtm(Sigma)*z;

% return

end

%%

function X = genLinearStateSequence(x_0, P_0, f, Q, K)
    % GENLINEARSTATESEQUENCE generates an N-long sequence of states using a 
    %    Gaussian prior and a linear Gaussian process model
    %
    % Input:
    %   x_0         [n x 1] Prior mean
    %   P_0         [n x n] Prior covariance
    %   f                   Function handle for 
    %   Q           [n x n] Process noise covariance
    %   K           [1 x 1] Number of states to generate
    %
    % Output:
    %   X           [n x N+1] State vector sequence
    
    % Number of states
    n = length(x_0);

    % Initialize X output
    X = zeros(n,K);
    
    % Sample initial state from the prior distribution
    X(:,1) = mvnrnd(x_0, P_0)';

    % Iterate to generate K samples
    for i=1:K
        % Propagate through motion model
        X(:,i+1) =  f(X(:,i)) + mvnrnd(zeros(n,1), Q)';
    end

end
%% 
function Y = genLinearMeasurementSequence(X, H, R)
    % GENLINEARMEASUREMENTSEQUENCE generates a sequence of observations of the state 
    % sequence X using a linear measurement model. Measurement noise is assumed to be 
    % zero mean and Gaussian.
    %
    % Input:
    %   X           [n x N+1] State vector sequence. The k:th state vector is X(:,k+1)
    %   H                     Function handle for measurement model
    %   R           [m x m]   Measurement noise covariance
    %
    % Output:
    %   Y           [m x N] Measurement sequence
    %

    % Number of measurements
    m = size(H,1);

    % State sequence includes x0, which does not generate an observation
    K = size(X,2) -1;

    % Initialize output
    Y = zeros(m,K);
    
    % Iterate to generate K samples
    for i=1:K
        % Measurement model: Y{k} = H*X{k} + r{k},   where r{k} ~ N(0,R)
        Y(:,i) = H(X(:,i+1)) + mvnrnd(zeros(m,1), R)';
    end
    
end
%%
function [x, P] = linearKalmanPrediction(x, P, A, Q)
    % LINEARPREDICTION calculates mean and covariance of predicted state
    %   density using a linear Gaussian model.
    %
    % Input:
    %   x           [n x 1] Prior mean
    %   P           [n x n] Prior covariance
    %   A           [n x n] State transition matrix
    %   Q           [n x n] Process noise covariance
    %
    % Output:
    %   x           [n x 1] predicted state mean
    %   P           [n x n] predicted state covariance
    %

    % prediction step: compute p(x_k | y_1:k-1) from p(x_k-1 | y_1:k-1)
    %
    % Use motion model for prediction 
    % \hat{x}_{k|k-1} = A_{k-1} * \hat{x}_{k-1|k-1}
    x = A * x;
    % Compute covariance of new prediction:
    % Cov(x_{k|k-1}) = Cov(A_{k-1} * x_{k-1|k-1} + q) = A*P*A' + Q
    P = A * P * A' + Q;

end

%%
function [X, P] = linearKalmanFilter(Y, x_0, P_0, f, Q, h, R)
    % KALMANFILTER Filters measurements sequence Y using a Kalman filter. 
    %
    % Input:
    %   Y           [m x N] Measurement sequence
    %   x_0         [n x 1] Prior mean
    %   P_0         [n x n] Prior covariance
    %   A           [n x n] State transition matrix
    %   Q           [n x n] Process noise covariance
    %   H           [m x n] Measurement model matrix
    %   R           [m x m] Measurement noise covariance
    %
    % Output:
    %   x           [n x N] Estimated state vector sequence
    %   P           [n x n x N] Filter error convariance
    %

    % Parameters
    N = size(Y,2);

    n = length(x_0);
    m = size(Y,1);

    % Data allocation
    x = zeros(n,   N +1);
    P = zeros(n,n, N +1);

    % filter
    x(:,1)   = x_0;
    P(:,:,1) = P_0;
    
    for i=1:N
        % prediction step: compute p(x_k | y_1:k-1) from p(x_k-1 | y_1:k-1)
        [x(:,i+1), P(:,:,i+1)] = linearKalmanPrediction(x(:,i), P(:,:,i), f, Q);

        % update step: compute p(x_k | y_1:k) from p(x_k | y_1:k-1)
        [x(:,i+1), P(:,:,i+1)] = linearKalmanUpdate(x(:,i+1), P(:,:,i+1), Y(:,i), h, R);
    end
    
    % exclude prior x0~N(x_0, P_0) from posterior
    X = x(:,2:end);
    P = P(:,:,2:end);
    
end
%%
function [x, P] = linearKalmanUpdate(x, P, y, H, R)
    % LINEARUPDATE calculates mean and covariance of predicted state
    %   density using a linear Gaussian model.
    %
    % Input:
    %   x           [n x 1] Prior mean
    %   P           [n x n] Prior covariance
    %   y           [m x 1] Measurement
    %   H           [m x n] Measurement model matrix
    %   R           [m x m] Measurement noise covariance
    %
    % Output:
    %   x           [n x 1] updated state mean
    %   P           [n x n] updated state covariance
    %

    % update step: compute p(x_k | y_1:k) from p(x_k | y_1:k-1)
    %
    % inovation mean
    v = y - H * x;
    % inovation covariance
    S = H * P * H' + R;
    % kalman gain
    K = P * H' / S;

    % updated state mean
    x = x + K * v;
    % updated state covariance
    P = P - K * S * K';

end
%%
function plotParticleFilter(X,Y,K,xf,Pf,xfp,Pfp,xfp_resam, Pfp_resam)
%PLOTPARTICLEFILTER Summary of this function goes here
%   Detailed explanation goes here


    figure()
    grid on; hold on; axis on

    % Plot true state
    plot(0:K,X,'k','LineWidth',2);

    % Plot measured
    plot(1:K,Y,':','LineWidth',2,'color','#808830');

    % Plot kalman filtered estimate
    plot(1:K,xf,'LineWidth',2,'color','#ED0120');

    % Plot particle filtered estimate NOT RESAMPLED
    plot(1:K,xfp,'LineWidth',2,'color','#AA142F');

    % Plot particle filtered estimate RESAMPLED
    plot(1:K,xfp_resam,'LineWidth',1,'color','#22AC30');

    % Show axis
    xline(0); yline(0);

    % Legend Title etc.
    legend('True state','Measured state',...
        'Kalman filtered state trajectory',...
        'Particle filtered','Particle filtered w/ resampling');
    xlabel('Timesteps k'); ylabel('x_k');
    ylabel('x_k');
    ylim([-15 10])

    % Return
    
end

%%
function plotParticleFilterErrorbar(X,Y,K,xf,Pf,xfp,Pfp,xfp_resam,Pfp_resam)
%PLOTPARTICLEFILTERERRORBAR Summary of this function goes here
%   Detailed explanation goes here


    figure()
    grid on; hold on

    % Plot true state
    plot(0:K,X,'k','LineWidth',2);

    % Plot Kalman filtered estimate w/ errorbar
    errorbar(1:K,xf,reshape(Pf,[1,length(Pf)]),'LineWidth',3,'color','#EDB120')

    % Plot Partice filtered estimate w/ errorbar
    errorbar(1:K,xfp,reshape(Pfp,[1,length(Pfp)]),'LineWidth',2,'color','#A2142F')

    % Plot Partice filtered estimate w/ errorbar w/ resampling
    errorbar(1:K,xfp_resam,reshape(Pfp_resam,[1,length(Pfp_resam)]),'LineWidth',1,'color','#77AC30')

    % Show axis
    xline(0); yline(0);

    % Legend Title etc.
    legend('True state',...
        'Kalman filtered state trajectory',...
        'Particle filtered','Particle filtered w/ resampling')
    xlabel('Timesteps k'); ylabel('x_k');
       ylim([-15 10])


end
%%

function plotPosteriorApproximation(xf, Pf, xfp, Pfp, xfp_resam, Pfp_resam, time_instances)

    %PLOTPOSTERIORAPPROXIMATION Summary of this function goes here
    %   Detailed explanation goes here

    % Create figure
    figure()

    % Define colors
    colors = {'#EDB120', '#A2142F', '#77AC30'};

    for i=1:3

        % change plot handle
        subplot(3,1,i)
        hold on; grid on

        mean = xf(time_instances(i));
        sigma = sqrt(Pf(:,:,time_instances(i)));
        x = linspace(mean-(4*sigma), (mean+4*sigma));
        y = pdf('Normal',x,mean,sigma);
        xlim([x(10) x(end-10)]);
     
        % Plot posterior from Kalman filter
        plot(x, y, 'color', colors{1}, 'Linewidth', 2)

        mean = xfp(time_instances(i));
        sigma = sqrt(Pfp(:,:,time_instances(i)));
        x = linspace(mean-(4*sigma), mean+(4*sigma));
        y = pdf('Normal',x,mean,sigma);

        plot(x, y, 'color', colors{2}, 'LineWidth', 2)

        mean = xfp_resam(time_instances(i));
        sigma = sqrt(Pfp_resam(:,:,time_instances(i)));
        x = linspace(mean-(4*sigma), mean+(4*sigma));
        y = pdf('Normal',x,mean,sigma);

        plot(x, y, 'color', colors{3}, 'LineWidth', 2)

        % Legend Title etc.
        legend('Kalman filtered posterior',...
        'Particle filtered posterior','Particle filtered w/ resampling posterior');
        xlabel('posterior mean');
        
    end

end

%%
function plotParticleTrajectory(Xp,Wp,K,N,X,xfp,Pfp,bResampl)
    %PLOTPARTICLETRAJECTORY Summary of this function goes here
    %   Detailed explanation goes here
    
    
    figure()
    hold on; grid on

    % Plot true state
    a = plot(1:K,X(1:end-1),'k','LineWidth',2);

    % Decide color
    if ~bResampl
        customColor = '#A2642F';
    else
        customColor = '#770C30';
    end

    % Plot Partice filtered estimate w/ errorbar
    b = errorbar(1:K,xfp,reshape(Pfp,[1,length(Pfp)]),'LineWidth',2,'color',customColor);

    % Plot particle traj
    for i=1:N
        p1 = plot(1:K,reshape(Xp(:,i,:),[1,K]),'-','color',customColor);
        p1.Color(4) = 0.2;
        hold on
    end


    % Show axis
    xline(0); yline(0);

    % plot attributes
    legend('True state','PF filtered','Particle paths')
    uistack(b, 'top')
    uistack(a, 'top')
    xlabel('Timesteps k'); ylabel('x_k');
    xlim([1 length(xfp)]);
    ylim([-15 15]);

end