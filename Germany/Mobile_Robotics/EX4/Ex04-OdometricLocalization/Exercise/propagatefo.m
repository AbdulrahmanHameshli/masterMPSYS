function [poseOut, covOut, arcPoints] = propagatefo(x, C, b)
    X0 = x(1);
    Y0 = x(2);
    Theta0 = x(3);
    sL = x(4);
    sR = x(5);

    dist = (sR + sL) / 2;
    deltaTheta  = (sR - sL) / b;

    Thetaout = Theta0 + deltaTheta;
    
    if abs(deltaTheta) < 1e-6
    
        Xout = X0 + cos(Thetaout) * dist;
        Yout = Y0 + sin(Thetaout) * dist;

    else

        Xout = X0 + (sin(Thetaout) - sin(Theta0)) * (dist/deltaTheta);
        Yout = Y0 - (cos(Thetaout)- cos(Theta0)) * (dist/deltaTheta);
       
    end
    
    poseOut = [Xout; Yout; Thetaout];
    
    Fx = eye(3);  
    

    
    Fu = [cos(Thetaout + deltaTheta/ 2), 0; 
      sin(Thetaout + deltaTheta/ 2), 0; 
      0, 1];

    covStart = C(1:3, 1:3);
    covU = C(4:5, 4:5);
   
    covOut = Fx * covStart * Fx' + Fu * covU * Fu';
    
    nArcPoints = 30;
    arcPoints = zeros(2, nArcPoints);
    
    if abs(deltaTheta) < 1e-6
    % Straight line case
        for i = 1:nArcPoints
            % Linear interpolation from start to end point
            arcPoints(1, i) = X0 + (i - 1) * dist / (nArcPoints - 1) * cos(Theta0);
            arcPoints(2, i) = Y0 + (i - 1) * dist / (nArcPoints - 1) * sin(Theta0);
        end
    else
        % Arc case
        for i = 1:nArcPoints
            thetaArc = Theta0 + (i - 1) * deltaTheta / (nArcPoints - 1);
            arcPoints(1, i) = X0 + (dist / deltaTheta) * (sin(thetaArc) - sin(Theta0));
            arcPoints(2, i) = Y0 - (dist / deltaTheta) * (cos(thetaArc) - cos(Theta0));
        end
    end


end
