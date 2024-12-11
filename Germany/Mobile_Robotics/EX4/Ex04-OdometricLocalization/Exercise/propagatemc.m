function [poseOut, covOut, samples] = propagatemc(nSamples, x, C, b)
    samples = zeros(3, nSamples); 
    
    
    for i = 1:nSamples
        x_noisy = x + chol(C) * randn(size(x));
        
        poseNoisy = x_noisy(1:3);
        sL_noisy = x_noisy(4);
        sR_noisy = x_noisy(5);
        
        deltaTheta = (sR_noisy - sL_noisy) / b;     % Change in orientation
        dist = (sL_noisy + sR_noisy) / 2;           % Average displacement
        
        thetaOut = poseNoisy(3) + deltaTheta;       % New orientation
        
        % Calculate new pose
        if abs(deltaTheta) < 1e-6
            xOut = poseNoisy(1) + cos(poseNoisy(3)) * dist;
            yOut = poseNoisy(2) + sin(poseNoisy(3)) * dist;
        else
            xOut = poseNoisy(1) + (sin(thetaOut) - sin(poseNoisy(3))) * (dist / deltaTheta);
            yOut = poseNoisy(2) - (cos(thetaOut) - cos(poseNoisy(3))) * (dist / deltaTheta);
        end
        
        % Store the propagated pose for this sample
        samples(:, i) = [xOut; yOut; thetaOut];
    end
    
    % Compute the mean pose and covariance from the samples
    poseOut = mean(samples, 2);                 % Average pose over all samples
    covOut = cov(samples');                     % Covariance of the sampled poses
end
