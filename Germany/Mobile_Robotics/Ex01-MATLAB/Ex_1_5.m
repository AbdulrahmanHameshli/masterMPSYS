N = 10000;
x = -1 + 2 * rand(N, 1); 
y = -1 + 2 * rand(N, 1); 

distances = sqrt(x.^2 + y.^2);

inliers = distances <= 1;
K = sum(inliers); 
pi_estimate = 4 * K / N;
error = abs(pi_estimate - pi);

figure;
hold on;
plot([-1 1 1 -1 -1], [-1 -1 1 1 -1], 'k-', 'LineWidth', 2);
scatter(x(inliers), y(inliers), 10, 'b', 'filled');  % Blue points
scatter(x(~inliers), y(~inliers), 10, 'r', 'filled');  % Red points
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'g-', 'LineWidth', 2);  % Green circle
axis equal;
axis([-1.2 1.2 -1.2 1.2]);
grid on;
hold off;
disp(["pi= " + num2str(pi_estimate) + ", error= " + num2str(error)])
