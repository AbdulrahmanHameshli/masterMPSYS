function plotlocalmeasurement(figureHandle, scan, beacons, config)
subplot(figureHandle); cla;    
title('local map: features');
drawrobot([0; 0; 0],'k',4);
plot(scan(1,:),scan(2,:),'+','MarkerSize',3,'Color',[.6 .6 .6]);

if ~isempty(beacons)
    plot(beacons(1,:),beacons(2,:),'d','Color',[1 .5 .0],'MarkerSize',10,'MarkerFaceColor',[1 .6 .1]);
end
axis equal; axis([-config.MAXR config.MAXR -config.MAXR config.MAXR]);
end

