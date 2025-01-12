function [particles, weights, localizationEstimate, logRunData] = particlefilterstep(particles, weights, deltaWheelAngles, globalMap, scan, config, logRunData)
    % Perform a single step of the particle filter localization algorithm
    % Inputs:
    %   particles - (3 x N) matrix of particle states [x; y; theta]
    %   weights - (1 x N) vector of particle weights
    %   deltaWheelAngles - [deltaLeft, deltaRight] wheel encoder angles
    %   globalMap - Map of the environment
    %   scan - Sensor measurement (e.g., laser scan)
    %   config - Configuration structure
    %   logRunData - Structure for logging runtime data
    % Outputs:
    %   particles - (3 x N) updated particle states
    %   weights - (1 x N) updated weights
    %   localizationEstimate - Estimated robot pose [x; y; theta]
    %   logRunData - Updated log structure

   
    particles = propagateparticles(particles, deltaWheelAngles, config);

    weights = updateparticleweights(particles, weights, globalMap, scan, config);
    
    Neff = 1 / sum(weights.^2);  

    if Neff < config.NEFFTHRESHOLD * size(weights, 2)
        [particles, weights] = resamplesystematic(particles, weights);
    end

    localizationEstimate = sum(particles .* weights, 2);

    logRunData.Estimated = [logRunData.Estimated, localizationEstimate];
end
