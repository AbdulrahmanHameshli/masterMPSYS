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
axis([-2 2.1 -3 2.1]);
hold on
hh1 = fill(X+L1,Y,'b');

hh_2 = fill(X,Y,'r');
axis square
axis([-2.1 2.1+offset -3 2]);
hold on
hh1_2 = fill(X+L1,Y,'b');

time = q1values.Time;
q11 = q1values.Data;
q12 = q2values.Data;

q21 = q1values1.Data;
q22 = q2values1.Data;

xend = L1*cos(q11)+L2*cos(q11+q12);
yend = L1*sin(q11)+L2*sin(q11+q12);

hend = plot(xend(1,1),yend(1,1));
hold on

for ii=1:length(time)
    aa = tic;

    RR = [cos(q11(ii)), -sin(q11(ii)); sin(q11(ii)), cos(q11(ii))];
    CC = RR*[X;Y];
    RR1 = [cos(q11(ii)+q12(ii)), -sin(q11(ii)+q12(ii)); sin(q11(ii)+q12(ii)), cos(q11(ii)+q12(ii))];
    DD = RR1*[X;Y];
    DD(1,:) = DD(1,:) + L1*cos(q11(ii));
    DD(2,:) = DD(2,:) + L1*sin(q11(ii));
    
    set(hh, 'XData', CC(1,:),  'YData', CC(2,:));
    set(hh1, 'XData', DD(1,:),  'YData', DD(2,:));
    set(hend, 'XData', xend(1:ii),  'YData', yend(1:ii))
    
    RR = [cos(q21(ii)), -sin(q21(ii)); sin(q21(ii)), cos(q21(ii))];
    CC = RR*[X;Y]+[offset;0];
    RR1 = [cos(q21(ii)+q22(ii)), -sin(q21(ii)+q22(ii)); sin(q21(ii)+q22(ii)), cos(q21(ii)+q22(ii))];
    DD = RR1*[X;Y]+[offset;0];
    DD(1,:) = DD(1,:) + L1*cos(q21(ii));
    DD(2,:) = DD(2,:) + L1*sin(q21(ii));
    
    set(hh_2, 'XData', CC(1,:),  'YData', CC(2,:));
    set(hh1_2, 'XData', DD(1,:),  'YData', DD(2,:));
    set(hend, 'XData', xend(1:ii),  'YData', yend(1:ii))
    
    drawnow
    bb = toc(aa);

    if ii > 1 
        TT = time(ii-1) - time(ii);
    end
end