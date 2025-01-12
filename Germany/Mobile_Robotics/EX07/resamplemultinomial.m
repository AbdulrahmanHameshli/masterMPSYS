% function [resampledParticles, resampledWeights] = resamplemultinomial(particles, weights)
%     % Resample particles using multinomial resampling
%     % Inputs:
%     %   particles - (3 x N) matrix of particle states [x; y; theta]
%     %   weights - (1 x N) vector of particle weights (normalized)
%     % Outputs:
%     %   resampledParticles - (3 x N) matrix of resampled particle states
%     %   resampledWeights - (1 x N) vector of resampled weights (uniform)
% 
%     % Number of particles
%     nParticles = size(particles, 2);
% 
%     % Compute the cumulative sum of weights
%     cumulativeSum = cumsum(weights);
% 
%     % Initialize resampled particles
%     resampledParticles = zeros(size(particles));
% 
%     % Perform multinomial resampling
%     for i = 1:nParticles
%         % Generate a random number in [0, 1]
%         r = rand();
% 
%         % Find the index of the first particle whose cumulative weight
%         % exceeds the random number
%         index = find(cumulativeSum >= r, 1);
% 
%         % Assign the corresponding particle to the resampled set
%         resampledParticles(:, i) = particles(:, index);
%     end
% 
%     % Assign uniform weights to resampled particles
%     resampledWeights = ones(1, nParticles) / nParticles;
% end



% Mobile Robotics Exercise
% Particle Filter Localization
%
% Particle Filter Resampling (Multinomial)
%   Computes the multinomial resampling in PF localization
%
% Inputs:
%   particles: set of particles (3xN matrix, where N is the number of particles)
%   weights: 1xN vector, weights for the particles
%
% Outputs:
%   newParticles: resampled particles (3xN matrix)
%   newWeights: resampled weights (1xN vector, uniform after resampling)

function [newParticles, newWeights] = resamplemultinomial(particles, weights)
    nParticles = size(particles, 2);

    weightSum = sum(weights);
    if weightSum == 0
        error('Sum of weights is zero. Check the weight computation.');
    end
    weights = weights / weightSum;
    cumulativeSum = cumsum(weights);
    newParticles = zeros(size(particles));
    newWeights = ones(1, nParticles) / nParticles; 

    for i = 1:nParticles
        r = rand();
        selectedIndex = find(cumulativeSum >= r, 1);
        newParticles(:, i) = particles(:, selectedIndex);
    end
end