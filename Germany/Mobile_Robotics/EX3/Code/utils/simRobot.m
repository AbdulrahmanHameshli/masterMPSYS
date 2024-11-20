function  simRobot(wheels,contour, dt, useKB)
%SIMROBOT Simulate a robot composed by a set of wheels
% wheels: a set of wheels
% contour: the contour of the robot (passthrough to drawrobot)
% dt: simulation timestep (Euler fwd)
% useKB: if true, use the keyboard to move the robot. The first input is
% bound to [W,S], the second to [A,D], the third (if it is needed) to
% [F,G]. If false, you can click on the axes to drive the robot to that
% point

% LIMITATIONS:
% 1) the wheels have to be specified in the following order: fixed,
% centered-steerable and caster wheels. 
% 2) Swedish wheels are not supported.
% 3) Additionally, the wheels have to be specified in a configuration coherent
% with "Structural properties and classification of kinematic and dynamic
% models of wheeled mobile robots", Campion et al. This implies, for
% example, that in a (2,1) robot the steerable wheel has to be in
% [x,y]==[0,0]. See section 4.A of the paper.
% 4) more than 2 centered steerable wheels are not supported.
% Every set of wheels provided in the exercise will be compliant to these
% limitations. see end of file if you want to remove those limitations.

% compute the type
[~,~,~,~,dm,ds] = computeConstraints(wheels);

type=[dm,ds];
if dm == -1 || ds == -1
    q = zeros(3,1);
else
    q = zeros(3+ds,1);
end
qc = zeros(sum(arrayfun(@(s) s.type == 2, wheels)),1); % configuration vector for caster wheels (kept separate)

% Set up the figure
fig = figure;
ax = axes('Parent', fig);
axis([-4 4 -4 4]);  % Define axis limits
axis square;
hold on;

% keyboard stuff
global keyState
keyState = struct('w', false, 'a', false, 's', false, 'd', false, 'q', false, 'xy', [1; 2], 'f', false, 'g', false);
set(gcf, 'KeyPressFcn', @(~, event) keyHandle(event, true));
set(gcf, 'KeyReleaseFcn', @(~, event) keyHandle(event, false));
set(ax, 'ButtonDownFcn', @mouseHandle);

t = 0.0;

disp("Press Q or Ctrl+C to quit:")
if useKB
    disp("use WASD + FG to move robot")
else
    disp("click on goal point")
end
do_simulation = true;
while (do_simulation)
    tic

    PG = plot(keyState.xy(1), keyState.xy(2), "k+");

    
    G = fwdKin(type, q, wheels);
    
    if useKB 
        u = velKB(keyState);
    else
        [u, zpred] = goToPos(type, q, keyState.xy, wheels);
        ZG = plot(zpred(1), zpred(2), "ko");
    end

    qdot = G*u(1:size(G,2));

    % BONUS Point: caster wheels kinematic
    if ~isempty(qc)
        qc_dot = casterKin(q(1:3), qdot(1:3), wheels(arrayfun(@(s) s.type == 2, wheels)));
        qc = qc + qc_dot*dt;
    end
    %

    % Euler fwd
    q = q + qdot * dt;

    % plot ICR
    ICR = getICR(q(1:3), wheels);

    ICRG = plot(ICR(1), ICR(2), "bo");

    % update the wheel state
    idx = 4;
    idxc = 1;
    for i=1:length(wheels)
        if wheels(i).type == 1 && ds ~= -1
            wheels(i).pose(3) = q(idx) ...
            + atan2(wheels(i).pose(2),wheels(i).pose(1)); % again, alpha

            % there is only one kind of robot where you have two steering
            % wheels, that is (1,2)
            idx = idx + 1;
        elseif wheels(i).type == 2 && ds ~= -1
            wheels(i).pose(3) = qc(idxc) ... 
            + atan2(wheels(i).pose(2),wheels(i).pose(1));
            idxc = idxc + 1;
        end
    end
    robotG = drawrobot(q(1:3), [0 0 0], contour, wheels);
    


    t = t + dt;

    drawnow;
    delta = dt-toc;
    if (delta >0)
        pause(delta);
    end
    
    if ~useKB
        delete(ZG)
    end
    delete(PG);
    delete(robotG);
    delete(ICRG)
    if keyState.('q')
        do_simulation = false;
    end
end
close(fig)

end

function keyHandle(event, val)
    global keyState
    keyState.(event.Key) = val;
end
function mouseHandle(src, event)
    global keyState
    mousePos = get(src, 'CurrentPoint');
    
    x = mousePos(1, 1);
    y = mousePos(1, 2);

    keyState.xy = [x;y];
    
end


function u = velKB(keyState)
u = zeros(3,1);
    k1 = 1.0;
    k2 = 1.0;
    k3 = 1.0;
    if keyState.w
        u(1) = k1;  
    end
    if keyState.s
        u(1) = -k1;
    end
    if keyState.a
        u(2) = k2;
    end
    if keyState.d
        u(2) = -k3;  
    end
    if keyState.f
        u(3) = k3;
    end
    if keyState.g
        u(3) = -k3;
    end
end



% TODO for future Fabio or other people 
% steps needed to remove the limitations:
% 1) just rearrange the list of wheels [easy]
% 2) those wheels have an additional parameter gamma that needs to be added 
% in wheel.params and the no-sliding constraint has to be computed 
% accordingly [easy]
% 3) Given a set of wheels, specified in an arbitrary frame of reference (T),
% we need to find the transformation into Campion et al. coordinates (C) for
% that type of robot. Then, the kinematic model proposed in the paper is
% valid for C. We then need to convert the velocities in C to T and 
% integrate them into the configuration vector expressed in my frame T.
% [hard]
% 4) We just need to identify which steering wheel is "master" and which is
% "redundant". In general, only one steering wheel can be "master" and all
% the other ones are redundant. The only exception is (1,2) robot which has
% have two independent steering wheels. After that, only master wheel
% orientations go into the configuration vector, the other orientations are
% determined analytically by constraining them to be in the ICR (see
% Ackermann steering and getICR.m) [hard]
% additional todos:
% We should move from using drawrobot at each frame (and deleting) to the
% use of hgtransforms
