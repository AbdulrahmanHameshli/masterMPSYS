function [newParticles, newWeights] = resamplesystematic(particles, weights)
    % Number of particles
    nParticles = size(particles, 2);

    % Normalize weights to ensure they sum to 1
    if sum(weights) == 0
        weights = ones(1, nParticles) / nParticles; % Reset to uniform weights
    else
        weights = weights / sum(weights);
    end

    % Step 1: Construct the cumulative distribution function (CDF)
    c = zeros(1, nParticles);
    c(1) = weights(1);
    for i = 2:nParticles
        c(i) = c(i-1) + weights(i);
    end

    % Step 2: Generate a random starting  point
    u1 = rand() / nParticles;

    % Step 3: Perform systematic resampling
    newParticles = zeros(size(particles));
    newWeights = ones(1, nParticles) / nParticles; % Equal weights after resampling
    i = 1;
    for j = 1:nParticles
        % Calculate the current threshold
        uj = u1 + (j-1) / nParticles;

        % Increment index i until uj <= c(i)
        while uj > c(i) + eps
            i = i + 1;
        end

        % Assign the particle and weight
        newParticles(:, j) = particles(:, i);
    end

    % Optional: Add diversity by injecting small noise
    small_noise = 1e-2; % Adjust noise level as needed
    newParticles = newParticles + randn(size(newParticles)) * small_noise;
end