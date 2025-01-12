function globalMap = loadandplotglobalmap(figureHandle, config)
    % Read in map
    eval(config.MAPNAME);       % Reads in matrix M by evaluating the .m-script

    globalMap = M;

    % Plot
    subplot(figureHandle);
    for i = 1:size(globalMap, 1)
        drawmapobject(globalMap(i,1), globalMap(i,2), globalMap(i,3), globalMap(i,4), globalMap(i,5));
    end
    config.mapXMin = min([globalMap(:,1); globalMap(:,3)]);
    config.mapXMax = max([globalMap(:,1); globalMap(:,3)]);
    config.mapYMin = min([globalMap(:,2); globalMap(:,4)]);
    config.mapYMax = max([globalMap(:,2); globalMap(:,4)]);
    drawreference([0; 0; 0],'',0.4,'k');
    axis equal; axis([config.mapXMin config.mapXMax config.mapYMin config.mapYMax]);
    title('global map');
end

