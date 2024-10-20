function plotcircle(x,y,rad,RGB)
theta = linspace(0, 2*pi, 100);

x = x + rad * cos(theta);
y = y + rad * sin(theta);

plot(x,y , 'LineWidth', 2,"Color",RGB);
axis equal;
xlabel('X-axis');
ylabel('Y-axis');
grid on;

end