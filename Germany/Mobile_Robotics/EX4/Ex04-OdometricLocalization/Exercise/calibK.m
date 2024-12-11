function calibK(K, nRuns, visualize)
    % calibK: Calibrates the growth coefficient K for odometry error propagation.
    % Input arguments:
    %   K: Initial under-approximated growth coefficient.
    %   nRuns: Number of simulation runs.
    %   visualize: Set to true for graphical visualization, false otherwise.

    chi2Threshold = chi2inv(0.95, 2); 

    for i = 1:nRuns
        OdometrySim; 

        pN = x(1:2); 
        Sigma_k = C(1:2, 1:2); 
        pGT = xgt(1:2); 

        deltaP = pGT - pN; 
        mahalanobisDistSquared = deltaP' * (Sigma_k \ deltaP); 

        if mahalanobisDistSquared < chi2Threshold
            fprintf('Run %d: K is acceptable. Mahalanobis distance^2 = %.3f\n', i, mahalanobisDistSquared);
        else
            fprintf('Run %d: K is too low. Increasing K. Mahalanobis distance^2 = %.3f\n', i, mahalanobisDistSquared);
            K = K * 2.5; 
            fprintf("K = %d", K)
        end
    end

    fprintf('Final calibrated K = %d\n', K);
end
