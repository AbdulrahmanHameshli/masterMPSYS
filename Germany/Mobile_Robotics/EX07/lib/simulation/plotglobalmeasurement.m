function hscanglobal = plotglobalmeasurement(figureHandle, scan, stateGT)
% Transform and plot simulated scan into global view/coordinates
    R = [cos(stateGT(3)) -sin(stateGT(3)); sin(stateGT(3)) cos(stateGT(3))];
    T = [stateGT(1); stateGT(2)]*ones(1,length(scan));
    scanGlobal = R*scan + T;
    subplot(figureHandle);
    
    hscanglobal = plot(scanGlobal(1,:),scanGlobal(2,:),'x','MarkerSize',3,'Color',[.1 .4 .9]);
end

