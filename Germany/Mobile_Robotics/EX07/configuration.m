% Simulation configuration
% Map
config.MAPNAME = 'MapWarehouse';
config.RANDOMMAP = 0;
config.NBEACONS = 20;         % number of beacons/point features in map
config.MAPSEED = 11;          % random generator seed for beacon locations
% Path
config.PATHFILENAME = 'AGVTestPath.txt';
%config.PATHFILENAME = 'SlalomPath.txt';
config.PLOTSTRIDE = 1;        % Only each nth step will be plotted
config.PAUSED = 0;            % Set to 1 for a step-by-step execution via keyboard
config.ANIMATE2FILE = 0;      % Set to 1 to generate an animated gif
% Differential drive kinematics
config.B = 0.40;              % wheelbase of diff. drive robot in [m]
config.RL = 0.1;              % radius of left wheel in [m]
config.RR = 0.1;              % radius of right wheel in [m]
config.BGT = 0.4001;          % wheelbase of diff. drive robot in [m]
config.RLGT = 0.0999;         % radius of left wheel in [m]
config.RRGT = 0.10002;        % radius of right wheel in [m]
% Odometry error modeling
config.KL = 0.00001;          % error growth coefficient for left wheel
config.KR = config.KL;               % error growth coefficient for right wheel
config.ENCERR = 0.1;       % ground truth angular displacement error, std in [rad]
% Laser scanner parameters and error modeling
config.ANGRESO = pi/30;      % range finder angular ANGRESOlution in [rad]
config.MAXR = 4.0;            % range finder perception radius in [m]
config.RSTD = 0.02;           % range standard deviation of range sensor. In [m]
config.CBEAC = 0.02*eye(2);   % beacon error model
config.PSENSORFAIL = 0;     % probability of a sensor failure
% Position controller frequency
config.DT = 0.3;
config.MANUALCONTROL = 0;
config.PLOT = 1;