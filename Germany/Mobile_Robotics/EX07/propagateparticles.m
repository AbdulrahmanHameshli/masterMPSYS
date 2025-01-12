function particles = propagateparticles(particles, deltaWheelAngles, config)
    % Propagate the set of particles using the given odometry motion model
    % Inputs:
    %   particles - (3 x N) matrix of particle states [x; y; theta]
    %   deltaWheelAngles - [deltaLeft, deltaRight] wheel encoder angles
    %   config - configuration structure with motion model parameters
    % Outputs:
    %   particles - (3 x N) updated particle states [x; y; theta]

    nParticles = size(particles, 2);

    noisyDeltaWheelAngles = zeros(2, nParticles);

    for i = 1:nParticles
        noisyDeltaWheelAngles(1, i) = deltaWheelAngles(1) + config.ENCERR * randn(1);
        noisyDeltaWheelAngles(2, i) = deltaWheelAngles(2) + config.ENCERR * randn(1);
    end
    for i = 1:nParticles
        state = particles(:, i);
        C = config.B / 2; 
        C = diag([1 ,1, 1]);
        [updatedState, ~, ~] = ododdforward(state, ...
                                           C, ...
                                           noisyDeltaWheelAngles(1, i), ...
                                           noisyDeltaWheelAngles(2, i), ...
                                           config.B, ...
                                           config.RL, ...
                                           config.RR, ...
                                           config.KL, ...
                                           config.KR);
        
        particles(:, i) = updatedState;
    end
end




% Mobile Robotics Exercise
% Particle Filter Localization
% 
% Particle Filter Particle Propagation
%   Computes the propagation of the particles with the motion model
% inputs:
%   particles: set of particles (3xnParticles)
%   deltaWheelAngles: increment of the wheels to reach the target point
%   config: configuration data of the session
% outputs:
%   newParticles: set of particles propagated with ododdforward
% 
% function [newParticles] = propagateparticles(particles, deltaWheelAngles, config)
%    deltaWheelAngles
%    noisydeltaWheelAngles =  deltaWheelAngles + config.ENCERR * randn(2,1);  
%    
%    newParticles = zeros(size(particles));
% 
%    % TODO
%     for i = 1:size(particles, 2)
%         P_state = particles(:,i);
%         [P_new_state,~,~] = ododdforward(P_state,[0,0,0;0,0,0;0,0,0],noisydeltaWheelAngles(1), ...
%             noisydeltaWheelAngles(2),config.B,config.RL,config.RR, config.KL,config.KR);
%     
%         newParticles(:, i) = P_new_state;
%    end
% end


