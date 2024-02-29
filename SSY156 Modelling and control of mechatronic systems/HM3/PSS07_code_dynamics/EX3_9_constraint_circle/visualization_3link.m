%% Script for the animation of the 3-link planar elbow manipulator.
% Run the simulation first.

h = 0.05;

x1 = 0;
y1 = 0;
x2 = L1;
y2 = 0;
x3 = L1;
y3 = -h;
x4 = 0;
y4 = -h;

X = [x1 x2 x3 x4];
Y = [y1 y2 y3 y4];

hh = fill(X,Y,'r');
axis square
axis([-3.1 3.1 -4 3]);
hold on
hh1 = fill(X+L1,Y,'b');
hh2 = fill(X+L2,Y,'g');

time = q1values.Time;
q1 = q1values.Data;
q2 = q2values.Data;
q3 = q3values.Data;


xend = L1*cos(q1)+L2*cos(q1+q2)+L3*cos(q1+q2+q3);
yend = L1*sin(q1)+L2*sin(q1+q2)+L3*sin(q1+q2+q3);

hend = plot(xend(1,1),yend(1,1));
hold on
%% 

for ii=1:length(time)
    aa = tic;

    RR = [cos(q1(ii)), -sin(q1(ii)); sin(q1(ii)), cos(q1(ii))];
    CC = RR*[X;Y];
    RR1 = [cos(q1(ii)+q2(ii)), -sin(q1(ii)+q2(ii)); sin(q1(ii)+q2(ii)), cos(q1(ii)+q2(ii))];
    DD = RR1*[X;Y];
    RR2 = [cos(q1(ii)+q2(ii)+q3(ii)), -sin(q1(ii)+q2(ii)+q3(ii)); sin(q1(ii)+q2(ii)+q3(ii)), cos(q1(ii)+q2(ii)+q3(ii))];
    EE = RR2*[X;Y];
    
    DD(1,:) = DD(1,:) + L1*cos(q1(ii));
    DD(2,:) = DD(2,:) + L1*sin(q1(ii));
    EE(1,:) = EE(1,:) + L1*cos(q1(ii)) + L2*cos(q1(ii)+q2(ii));
    EE(2,:) = EE(2,:) + L1*sin(q1(ii)) + L2*sin(q1(ii)+q2(ii));
    
    set(hh, 'XData', CC(1,:),  'YData', CC(2,:));
    set(hh1, 'XData', DD(1,:),  'YData', DD(2,:));
    set(hh2, 'XData', EE(1,:),  'YData', EE(2,:));
    set(hend, 'XData', xend(1:ii),  'YData', yend(1:ii))
    drawnow
    bb = toc(aa);

    if ii > 1
        TT = time(ii-1) - time(ii);
    end
end
hold off