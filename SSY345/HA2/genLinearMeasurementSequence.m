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