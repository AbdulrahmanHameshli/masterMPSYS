function [scanPolar, beaconsPolar] = generatemeasurement(stateGT, globalMap, config)
    % Perform ray tracing and simulate noisy sensor
    [Gs,Gp] = Box2G(globalMap);      % cast box data structure into old Gx-structures
    [scanPolar, beaconsPolar] = RayTrace(stateGT(1:3),Gs,Gp,config.ANGRESO,config.MAXR);
    % Add zero-mean Gaussian noise in radial direction to points and beacons
    scanPolar(:,2) = scanPolar(:,2) + config.RSTD*randn(length(scanPolar(:,2)),1);
    if ~isempty(beaconsPolar)
        beaconsPolar(:,2) = beaconsPolar(:,2) + config.RSTD*randn(length(beaconsPolar(:,2)),1);
    end
end

