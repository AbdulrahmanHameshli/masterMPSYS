function [newParticles, newWeights] = resamplekld(particles, weights)
    % Inputs:
    % - particles: set of particles (3 x nParticles)
    % - weights: weights for the particles (1 x nParticles)
    % Outputs:
    % - newParticles: resampled particles
    % - newWeights: resampled weights (equal weights after resampling)
    
    % Parameters for KLD resampling
    epsilon = 0.10;  % KLD error bound (tunable)
    delta = 0.1;    % Confidence bound (tunable, 99%)
    Nmin = 3;       % Minimum number of particles
    numParticles = size(particles, 2);
    
    % Initialize variables
    n = 1;  % Number of resampled particles
    k = 0;  % Number of occupied bins
    Na = 100;  % Adaptive number of particles
    newParticles = [];  % Resampled particles
    bins = containers.Map('KeyType', 'char', 'ValueType', 'double');  % Bin tracking
    
    % Precompute z_{1-delta} for KLD formula (inverse CDF of normal distribution)
    z1_delta = norminv(1 - delta);
    
    % Normalize weights if not already normalized
    weights = weights / sum(weights);
    
    % Resampling loop
    while (n < Na || n < Nmin)
        % Sample particle index using weights
        index = randsample(1:numParticles, 1, true, weights);
        sampledParticle = particles(:, index);
        newParticles = [newParticles, sampledParticle];  % Add to resampled set
        

                binKey = assignbin(sampledParticle);  
        if ~isKey(bins, binKey)
            bins(binKey) = 1;  
            k = k + 1;
            
            if k > 1
                    z_factor = (1 - (2 / (9 * (k - 1))) + sqrt((2 / (9 * (k - 1))) * z1_delta));
                    Na = (k - 1) / (2 * epsilon) * z_factor^3;
                
            end
                    % Increment resampled particle count
        end
        n = n + 1;

    end
    
    
    % Set weights to uniform after resampling
    newWeights = ones(1, size(newParticles, 2)) / size(newParticles, 2);
end

function binKey = assignbin(particle)
    % Function to assign a particle to a bin (discretization of particle space)
    % Inputs:
    % - particle: [x; y; theta] of the particle
    % Outputs:
    % - binKey: Unique identifier for the bin
    
    % Bin resolution (tunable)
    binResolution = 0.5;  % Adjust based on your map size and resolution
    
    % Discretize x, y, and theta
    xBin = floor(particle(1) / binResolution);
    yBin = floor(particle(2) / binResolution);
    thetaBin = floor(particle(3) / (pi / 4));  % 8 bins for theta (-pi to pi)
    
    % Create a unique key for the bin (string concatenation)
    binKey = sprintf('%d_%d_%d', xBin, yBin, thetaBin);
end
