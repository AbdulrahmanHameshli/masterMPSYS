% Mobile Robotics Exercise

% Particle Localization

% Main Script

%% Prepare Session
clear all;
close all;
clc
% format longEng
restoredefaultpath
addpath(genpath("lib/"))

% Load configuration as "config" variable
run("configuration.m")

% Particle localization (default) configuration
config.NINITIALPARTICLES = 100;
config.SAMPLEGT = 1 ; % Set to 1 for particle initialization at gt start
config.NEFFTHRESHOLD = 0.66;
config.RESAMPLINGALGORITHM = "systematic"; % can be "multinomial", "systematic", "kld" and "none"

% Adapt Configuration (optional)
config.USEPRECOMPUTEDLIKELIHOODFIELD = 0;

% Run the particle filter localization
disp("PF Localization - default values")
runpflocalization(config);

%% Runs for different PF configurations

% TODO for task 07.4


%% Function to run the PF localization session
function logRunData = runpflocalization(config)
    %% Prepare Figure Window and Simulation
    
    
    % Initialize figure
    figure(1); clf;

    % Subplot for local data
    f1 = subplot(2,2,1); cla; hold on; box on;  % 3rd input of subplot refers to mxn grid
    % Subplot for global data
    f2 = subplot(2,2,2); cla; hold on; box on;
    
    f3 = subplot(2, 2, [3, 4]); cla; hold on; box on;


    [globalMap, config] = loadandplotglobalmap(f2, config);
    
    %% Main Loop
    % Read in path and initialize history
    path = load(config.PATHFILENAME);
    path = path';
    nsteps = size(path,2);
    
    % Initialize simulation state and control
    stateGT = path(:,1);
    deltaWheelAngles = [0, 0];
    
    % Initialize particles (poses) and plot them
    if config.SAMPLEGT
        % Pose at the ground truth starting position
        particles = repmat(path(:,1),1,config.NINITIALPARTICLES);
    else
        % Pose uniformly distributed in operation space
        particles = samplefromfreespace(config.mapXMin, config.mapXMax, config.mapYMin, config.mapYMax, -pi, pi, config.NINITIALPARTICLES);
    end
    subplot(f2);
    hParticleScatter = scatter(particles(1, :), particles(2, :), "blue", "filled", 'MarkerFaceAlpha',0.25);
    % Initialize weights to 1
    weights = ones(1,config.NINITIALPARTICLES);
    
    % Initialize logging (could be extended with task 07.4)
    logRunData.stateGT = [];
    logRunData.Estimated = [];
    
    running=true;
    istep = 0;
    
    % Start execution timer
    tic
    % Iterate


    while running
        istep = istep +1;
        %% Generate Ground Truth Robot State For The Given Path
        [stateGT, deltaWheelAngles] = generatestategt(stateGT, deltaWheelAngles, path, istep, config);
        logRunData.stateGT = [logRunData.stateGT, stateGT];
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
    
        %% User Specific
    
        [particles, weights, localizationEstimate, logRunData] = particlefilterstep(particles, weights, deltaWheelAngles, globalMap, scan, config, logRunData);
    
    
%% Resample particles (this part depends on your existing resampling logic)
       if config.RESAMPLINGALGORITHM ~= "none"            
            % Count occurrences of identical particles
            [uniqueParticles, ~, idx] = unique(particles', 'rows', 'stable');
            counts = histcounts(idx, 1:max(idx) + 1);

            maxOccurrences(istep) = max(counts);
        end

        %% Finalize Cycle
        % Update figure with plotted data
        if config.PLOT
            subplot(f2)
            
            % Delete previous particle scatter plot and quiver arrows (if they exist)
            if exist('hParticleScatter', 'var') && isvalid(hParticleScatter)
                delete(hParticleScatter)
            end

            if exist('hParticleQuiver', 'var') && isvalid(hParticleQuiver)
                delete(hParticleQuiver)
            end
            
            % Plot particles as blue dots
            hParticleScatter = scatter(particles(1, :), particles(2, :), "blue", "filled", 'MarkerFaceAlpha', 0.25);
            
            % Add arrows for particle directions
            arrowLength = 0.5;  % Adjust arrow length as needed
            dx = arrowLength * cos(particles(3, :));  % X-direction component of the arrows
            dy = arrowLength * sin(particles(3, :));  % Y-direction component of the arrows
            hold on;  % Keep scatter plot and arrows on the same plot
            hParticleQuiver = quiver(particles(1, :), particles(2, :), dx, dy, 0, 'r', 'LineWidth', 1, 'MaxHeadSize', 0.5);
            
            drawrobot(localizationEstimate, 'r', 4);

            subplot(f3);
            errors = abs(logRunData.stateGT - logRunData.Estimated); % Assuming errors is a matrix
            hold on;
            plot(1:size(errors, 2), errors(1, :), 'b', 'LineWidth', 1.5); % Plot first error type in blue
            plot(1:size(errors, 2), errors(2, :), 'r', 'LineWidth', 1.5); % Plot second error type in red
            plot(1:size(errors, 2), errors(3, :), 'g', 'LineWidth', 1.5); % Plot third error type in green
            hold off;
            xlabel('Step');
            ylabel('Absolute Error');
            title('Localization Error Over Time');
            legend({'X error', 'Y error', 'Theta error'}); % Add legend for clarity
            grid on;

            %             scatter(1:length(errors), errors(2));
%             scatter(1:length(errors), errors(3));

            % Draw robot with current estimate
            
            % Update figure with plotted data    
            drawnow
        end

    
        if config.PAUSED
            waitforbuttonpress;
        end
    
        if ~config.MANUALCONTROL && istep>=nsteps
            running = false;
        end
    end
    meanMaxOccurrences = mean(maxOccurrences)
    meanLocalizationErrorX = mean(errors(1))
    meanLocalizationErrorY = mean(errors(2))
    meanLocalizationErrorTheta = mean(errors(3))
    

end
