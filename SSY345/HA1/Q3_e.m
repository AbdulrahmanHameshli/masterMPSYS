clc
clear


x = rand(100);
% x = unifpdf(x);
% r = normpdf(normrnd(0,1, [10,10]));
r = normrnd(0,1, [100,100]);


h = @(x)(2*x);


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
