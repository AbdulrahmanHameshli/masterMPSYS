

function newWeights = updateparticleweights(particles, weights, globalMap, scan, config)
    sigma = 0.02;  
    
    nParticles = size(particles, 2);
    Alpha_rand = 0.2;
    Alpha_hit = 0.8;
    
    newWeights = zeros(1, nParticles);
    
    for i = 1:nParticles
        xParticle = particles(1, i);
        yParticle = particles(2, i);
        thetaParticle = particles(3, i);
        
        particleLikelihood = 1; 
        
        for j = 1:size(scan, 2)
            xScan = scan(1, j);
            yScan = scan(2, j);
            
            
                thetaZ = atan2(yScan, xScan);

                Z = sqrt(xScan^2 + yScan^2);

                if Z < config.MAXR
                    scanX = xParticle + Z * cos(thetaZ + thetaParticle);
                    scanY = yParticle + Z * sin(thetaZ + thetaParticle);
                    

                    distance = closestobstacledistance(scanX, scanY, globalMap);

                    Phit = (1 / sqrt(2 * pi * sigma^2)) * exp(-(distance^2) / (2 * sigma^2));
                    Prand = 1 / config.MAXR;

                    particleLikelihood = particleLikelihood * (Alpha_hit * Phit + Alpha_rand * Prand);

                end

        end

        newWeights(i) = particleLikelihood ;
    end
    
    newWeights = newWeights/sum(newWeights);
%     if totalWeight == 0
%         % Reset to uniform weights if total weight is zero
%         newWeights = ones(1, nParticles) / nParticles;
%     else
%         newWeights = newWeights / totalWeight;
%     end
end
