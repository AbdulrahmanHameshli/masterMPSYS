% Script to Visualize Odometry Error Modeling
%
% Script simulates a mobile robot with a differential drive kinematics
% that incrementally determines its pose by wheel odometry. Systematic
% and non-systematic error parameters can be varied and their effect
% can be visualized

% v.1.0, 09.2024, Kai Arras, Uni Stuttgart


% Parameters
MAPNAME = 'Warehouse2Map';
PATHFILENAME = 'AGVTestPathManhattan.txt';
PLOTSTRIDE = 2;        % Only each nth step will be plotted
PLOTGT = 1;            % Set to 1 to plot ground truth robot
% Differential drive kinematics
B = 0.40;              % wheelbase of diff. drive robot in [m]
RL = 0.100;            % radius of left wheel in [m]
RR = 0.1;              % radius of right wheel in [m]
BGT = 0.401;             % wheelbase of diff. drive robot in [m]
RLGT = 0.1001;            % radius of left wheel in [m]
RRGT = 0.0999999;            % radius of right wheel in [m]
% Odometry error modeling

% K = 0.000001;          % error growth coefficient for left and right wheel
% commented because we calibrate it

ENCERR = 0.001;        % ground truth angular displacement error, std in [rad]
% Odometry frequency (related to the regulation controller frequency)
DT = 0.05;


%% Read and Plot Global Map 

% Read in path
rpath = load(PATHFILENAME);
rpath = rpath';
nsteps = size(rpath,2);

if visualize
    % Prepare Figure 
    figure(1); clf; hold on; axis off;
    % Read in map
    eval(MAPNAME);       % Reads in matrix M by evaluating the .m-script
    for i = 1:size(M, 1)
      h = drawmapobject(M(i,1), M(i,2), M(i,3), M(i,4), M(i,5));
      set(h,'LineWidth',1.3)
    end
    xmin = min([M(:,1); M(:,3)]); xmax = max([M(:,1); M(:,3)]);
    ymin = min([M(:,2); M(:,4)]); ymax = max([M(:,2); M(:,4)]);
    drawreference([0; 0; 0],'',0.4,'k');
    axis equal; axis tight; 
end

%% Main Loop
xgt = rpath(:,1); % initial state

x = xgt;          % initial state prediction
C = .0001*eye(3);    % initial state covariance


for istep = 1:nsteps
  plotthisstep = ~mod(istep,PLOTSTRIDE);

  % Generate Path and Simulate Odometry

  if istep > 1
    % Use position control to reach next via point
    [xctrl,vctrl] = positioncontrol(xgt,rpath(:,istep),1,DT,B);
    nctrl = size(vctrl,2);
    % Calculate angular wheel increments
    angdiff = DT*vctrl ./ repmat([RL; RR],1,nctrl);

    for i = 2:nctrl-1
      % Ground Truth forward simulation
      dphilgt = angdiff(1,i);
      dphirgt = angdiff(2,i);
      [xgt,~,~] = ododdforward(xgt,zeros(3),dphilgt,dphirgt,BGT,RLGT,RRGT,0,0);
      
      % add noise to encoder readings
      dphil = dphilgt + randn(1)*ENCERR;
      dphir = dphirgt + randn(1)*ENCERR;
      
      % student may not have done the task
      %[x,C,~] = propagatefo([x;dphil*RL; dphir*RR],...
      %                        blkdiag(C,diag([K*abs(dphil*RL), K*abs(dphir*RR)])),...
      %                        B);
      [x,C,~] = ododdforward(x, C, dphil, dphir, B, RL, RR, K, K);
    end
  end

  if plotthisstep && visualize
    drawrobot(x,[.9 .1 .1],4);
    drawprobellipse(x,C,0.95,[.9 .1 .1]);
    if PLOTGT
      drawrobot(xgt,'k',4);
    end
    drawnow;
  end
end

