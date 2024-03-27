clc
clear

x = rand(1000);
% x = unifpdf(x);
% r = normpdf(normrnd(0,1, [1000,1000]));
r = normrnd(0,1, [1000,1000]);
h = @(x)(2*x^2);
y = h(x) + r;


figure(1);
histogram(x)
    
figure(2);
histogram(h(x))

figure(3);
histogram(r)

figure(4)
histogram(y)

disp("mean")
    
mean(var(y))
mean(var(h(x)))


% mean(mean(h(x)))
% mean(mean(y))

[Y_given_X, Y_values] = ksdensity(y, x);
