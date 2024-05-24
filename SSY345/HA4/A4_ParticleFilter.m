clc;
clear;

% Set prior
sigma = 2;
x_0 = [0 1]';
P_0 = [sigma^2 0; ...
       0 sigma^2];
n = size(x_0,1);

% Number of time steps
K = 20;

% Models
A = [1 0.1; 0 1];
Q = [0 0; 0 0.5];
H = [1 0];
R = 1;

m = 1;

% Generate state and measurement sequences
X = zeros(n,K);
Y = zeros(m,K);
   
q = mvnrnd([0 0], Q, K)';
r = mvnrnd(zeros(1,m), R, K)';
x_kmin1 = x_0;
for k = 1:K
    xk = f(x_kmin1,A) + q(:,k);
    X(:,k) = xk;
    x_kmin1 = xk;
    
    Y(:,k) = h(xk, H) + r(:,k);
end

% Run PF filter with and without resampling
N = 20000;
proc_f = @(X_kmin1) (f(X_kmin1, A));
meas_h = @(X_k) (h(X_k, H));
plotFunc = @(k, Xk, Xkmin1, Wk, j) (0); % Define dummy function that does nothing

[xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Y, proc_f, Q, meas_h, R, ...
                              N, false, plotFunc);
                          
[xfpr, Pfpr, Xpr, Wpr] = pfFilter(x_0, P_0, Y, proc_f, Q, meas_h, R, ...
                                  N, true, plotFunc);



function X_k = f(X_kmin1, A)
%
% X_kmin1:  [n x N] N states vectors at time k-1
% A:        [n x n] Matrix such that x_k = A*x_k-1 + q_k-1
    X_k = A*X_kmin1;
end

function H_k = h(X_k, H)
%
% X_k:  [n x N] N states
% H:    [m x n] Matrix such that y = H*x + r_k
    H_k = H*X_k;
end

function [xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Y, proc_f, proc_Q, meas_h, meas_R, ...
                             N, bResample, plotFunc)
%PFFILTER Filters measurements Y using the SIS or SIR algorithms and a
% state-space model.
%
% Input:
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   Y           [m x K] Measurement sequence to be filtered
%   proc_f      Handle for process function f(x_k-1)
%   proc_Q      [n x n] process noise covariance
%   meas_h      Handle for measurement model function h(x_k)
%   meas_R      [m x m] measurement noise covariance
%   N           Number of particles
%   bResample   boolean false - no resampling, true - resampling
%   plotFunc    Handle for plot function that is called when a filter
%               recursion has finished.
% Output:
%   xfp         [n x K] Posterior means of particle filter
%   Pfp         [n x n x K] Posterior error covariances of particle filter
%   Xp          [n x N x K] Non-resampled Particles for posterior state distribution in times 1:K
%   Wp          [N x K] Non-resampled weights for posterior state x in times 1:K

% Your code here, please. 
% If you want to be a bit fancy, then only store and output the particles if the function
% is called with more than 2 output arguments.
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

    function [X_k, W_k] = pfFilterStep(X_kmin1, W_kmin1, yk, proc_f, proc_Q, meas_h, meas_R)
        X_k = mvnrnd(proc_f(X_kmin1)', proc_Q)';
        Wy = mvnpdf(yk', meas_h(X_k)', meas_R)';
        
        W_k = W_kmin1 .* Wy;
        W_k = W_k / sum(W_k);
    end