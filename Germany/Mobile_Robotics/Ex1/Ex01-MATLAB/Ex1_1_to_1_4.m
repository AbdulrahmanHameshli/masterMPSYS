%A
a = [1 2 3 4 5];
b = [0;1 ;3; 6; 10];
disp(['Transpose of b = ', num2str(b')])
Suma= sum(a)
Diffb= diff(b)

%B
% a*a does not work becouse of wron dim multiplcation 5x1 x 5x1. 

a.*a

a.^3

dot(a,b)

M= a'*b'

%C
who
%D
M(2,4)
M(1:2:5,3:end)
%E
det(M)
inv(M)
% the result is inf becouse M is singular matrix

%F
M(M > 9) = -1
%G
size(a)
size(b)
size(M)

%G
ones(size(M))
randn(size(M))

%1.3
%A,B,C
x = linspace(-4,4);
sin(x)
cos(x)
atan(x)
y = x + 0.3*x.^2 - 0.05*x.^3;
xlabel("Linespace")
ylabel("Value")
title("value over linespace")
legend("y")
plot(x,y,"r")

%1.4

%A

plotcircle(1,2,3)

function plotcircle(x,y,rad)
theta = linspace(0, 2*pi, 100);

x = x + rad * cos(theta);
y = y + rad * sin(theta);

plot(x,y, 'LineWidth', 4);
axis equal;
xlabel('X-axis');
ylabel('Y-axis');
grid on;

end
