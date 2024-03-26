
f_1 = @(x)[3*x];
f_2 = @(x)(x.^3 + 300);
[mu_y, Sigma_y, y_s ,x_s]= approxGaussianTransform(0,2,f_2,500000);

mu_y
Sigma_y

figure(1)
histogram(y_s);
xlabel("Y_s")

figure(2)
histogram(x_s);
xlabel("X_s")




% % True  values:
% disp('the true values is');
% [mu_y, Sigma_y] = affineGaussianTransform(0,1, 3, 0)




function [mu_y, Sigma_y, y_s , x_s] = approxGaussianTransform(mu_x, Sigma_x, f, N)
%approxGaussianTran
% 
% sform takes a Gaussian density and a transformation 
%function and calculates the mean and covariance of the transformed density.
%
%Inputs
%   MU_X        [m x 1] Expected value of x.
%   SIGMA_X     [m x m] Covariance of x.
%   F           [Function handle] Function which maps a [m x 1] dimensional
%               vector into another vector of size [n x 1].
%   N           Number of samples to draw. Default = 5000.
%
%Output
%   MU_Y        [n x 1] Approximated mean of y.
%   SIGMA_Y     [n x n] Approximated covariance of y.
%   ys          [n x N] Samples propagated through f


if nargin < 4
    N = 5000;
end
%Your code here

x_s = mvnrnd(mu_x, Sigma_x, N)';
y_s = f(x_s);
mu_y = mean(y_s, 2);
Sigma_y = cov(y_s');


end



function [mu_y, Sigma_y] = affineGaussianTransform(mu_x, Sigma_x, A, b)
%affineTransformGauss calculates the mean and covariance of y, the 
%transformed variable, exactly when the function, f, is defined as 
%y = f(x) = Ax + b, where A is a matrix, b is a vector of the same 
%dimensions as y, and x is a Gaussian random variable.
%
%Input
%   MU_X        [n x 1] Expected value of x.
%   SIGMA_X     [n x n] Covariance of x.
%   A           [m x n] Linear transform matrix.
%   B           [m x 1] Constant part of the affine transformation.
%
%Output
%   MU_Y        [m x 1] Expected value of y.
%   SIGMA_Y     [m x m] Covariance of y.

%Your code here

mu_y = A * mu_x + b;
Sigma_y = A * Sigma_x *A';
end