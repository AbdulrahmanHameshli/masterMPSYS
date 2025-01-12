% Mobile Robotics Exercise
%
% EKF Localization
%
% Main Script

%% Prepare Session
clear all;
close all;
restoredefaultpath
addpath(genpath("lib/"))

% Load configuration as "config" variable
run("configuration.m")

% Adapt Configuration (optional)
config.MANUALCONTROL = 0;
config.PAUSED = 0;

%% Run the session
runsession(config)

%% Function to run the PF localization session
function runsession(config)
%% Prepare Figure Window and Simulation
% Initialize figure
figure(1); clf;
% Subplot for local data
f1 = subplot(3,8,[1:3 9:11 17:19]); cla; hold on; box on;  % 3rd input of subplot refers to mxn grid
% Subplot for global data
f2 = subplot(3,8,[4:8 12:16 20:24]); cla; hold on; box on;

globalMap = loadandplotglobalmap(f2, config);


%% Main Loop
% Read in path and initialize history
path = load(config.PATHFILENAME);
path = path';
nsteps = size(path,2);

% Initialize simulation state and control
stateGT = path(:,1);
deltaWheelAngles = [0, 0];

% Initial state estimate at ground truth
state = stateGT;
% Initial state uncertainty covariance
C = 0.0001*eye(3);

running=true;
istep = 0;
% Iterate over the path
while running
    istep = istep +1;
    %% Generate Ground Truth Robot State For The Given Path
    [stateGT, deltaWheelAngles] = generatestategt(stateGT, deltaWheelAngles, path, istep, config);
    % Plot the ground truth robot state
    subplot(f2);
    drawrobot(stateGT,'k',4, 0.25, 0.25);

    %% Simulate Beacon Measurements
    if rand(1) > config.PSENSORFAIL
        % Generate noisy measurement
        [scanPolar, beaconsPolar] = generatemeasurement(stateGT, globalMap, config);
        % Transform to local cartesian coordinates
        [xvec,yvec] = pol2cart(scanPolar(:,1),scanPolar(:,2));
        scan = [xvec,yvec]';
        beacons = []';
        if ~isempty(beaconsPolar)            
            [xvec,yvec] = pol2cart(beaconsPolar(:,1),beaconsPolar(:,2));
            beacons = [xvec,yvec]';
        end        

        %%% Plot features
        plotlocalmeasurement(f1, scan, beacons, config)
        
        if exist('hScanGlobal','var'), delete(hScanGlobal); end
        hScanGlobal = plotglobalmeasurement(f2, scan, stateGT);
    end

    %% EKF Localization Implementation

    % TODO update "state" and "C" using EKF
% ------------------ Prediction Step ------------------

   noisydeltaWheelAngles =  deltaWheelAngles + config.ENCERR * randn(2,1);
    [state,C,~] = ododdforward(state,C,noisydeltaWheelAngles(1), ...
        noisydeltaWheelAngles(2),config.B,config.RL,config.RR,config.KL,config.KR);
  
% ------------------ Correction Step ------------------

    [totalInnovation, totalH, totalR] = predictandmatch(state, C, beacons, globalMap, config);

    % If valid beacons are matched, perform EKF update
    if ~isempty(totalInnovation)

        S = totalH * C * totalH' + totalR;  
        K = C * totalH' / S;  
  
        state = state + K * totalInnovation;
        C = (eye(size(C)) - K * totalH) * C;
    end


     
    subplot(f2);
    drawrobot(state,'r',4, 0.2, 0.2);
    drawprobellipse(state, C, 0.99, [1, 0, 0]);

    %% Finalize Cycle
    % Update figure with plotted data
    drawnow;

    if config.PAUSED
        waitforbuttonpress;
    end

    if ~config.MANUALCONTROL && istep>=nsteps
        running = false;
    end
end
end



