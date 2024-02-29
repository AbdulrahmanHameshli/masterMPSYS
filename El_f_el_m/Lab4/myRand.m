l = 0;
for i = 10
    x = -pi:0.01:pi;
    y=  sin(x-l);
    plot(x,y);
    l = l + 90;
    radians = deg2rad(l);
end