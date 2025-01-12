%POSITIONCONTROLLER   Position controller for differential drive robots
%   [XVEC, VVEC] = POSITIONCONTROLLER(XSTART, XEND, DIR, DT, B) generates 
%   and returns the trajectory which brings a differential drive robot 
%   with wheelbase B from the start pose XSTART to the goal pose XEND.
%   Calls the control function PosControlStep every DT seconds.
%
%   Where:
%   xstart : 3x1 initial robot pose vector [x; y; theta]
%   xend   : 3x1 goal pose vector [x; y; theta]
%   dir    : direction of motion (1 forward, -1 backward, 0 automatic)
%   dt     : time in [s]. Control frequency is 1/dt
%   b      : wheelbase of robot
%   xvec   : robot trajectory. 3xn matrix of n 3x1-poses
%   vvec   : 2xn matrix of wheel speeds: left (1st row), right (2nd row)
%
%   See also PosControlStep

% Based on the position controller by Allesandro Astolfi, ETHZ
% Matlab version: 1998 Remy Blank EPFL-ASL
% Modified 2000.10.09 Gilles Caprari EPFL-ASL
% v.1.0, Kai Arras, EPFL-ASL
% v.1.1, Jan Weingarten, Kai Arras: b argument added
% v.1.2, Kai Arras, CAS-KTH: suboptimal odometry expressions improved (dSd/2)


function [xvec, speedvec] = positioncontrol(xstart, xend, dir, deltaT, b)

if (size(xstart,1)*size(xstart,2) == 3) && (size(xend,1)*size(xend,2) == 3),
  if size(xstart) == [1,3], xstart = xstart'; end;
  if size(xend)   == [1,3], xend   = xend';   end;
  
  % Initialize variables
  sl = 0;
  sr = 0;
  oldSl = 0;
  oldSr = 0;
  xvec = [];				         % pose vectors for trajectory
  speedvec = [0; 0];         % velocities during trajectory
  encoders = [0; 0];
  t = 0;                     % initialize timer
  eot = 0;                   % initialize end-of-trajectory flag
  xcurrent = xstart;         % current pose equals start pose
  while ~eot,
    
    % Calculate distances for both wheels
    dSl = sl-oldSl;						
    dSr = sr-oldSr;
    dSm = (dSl+dSr)/2;
    dSd = (dSr-dSl)/b;
    
    % Integrate robot position
    xcurrent(1) = xcurrent(1) + dSm*cos(xcurrent(3)+dSd/2);
    xcurrent(2) = xcurrent(2) + dSm*sin(xcurrent(3)+dSd/2);
    %xcurrent(3) = normangle(xcurrent(3) + dSd, -pi);
    xcurrent(3) = setangletorange(xcurrent(3) + dSd, -pi);
    
    [vl, vr, eot] = posctrlstep(t, xcurrent, xend, dir, b);
    
    speeds = [vl; vr];
    % Add current values to trajectory
    speedvec = [speedvec, speeds];
    xvec     = [xvec, xcurrent];
    
    % Increase timer
    t = t + deltaT;
    
    % Increase accumulated encoder values
    encoders = encoders + speeds*deltaT;			% simulated encoders of robot

    % Keep track of previous wheel positions
    oldSl = sl;
    oldSr = sr;
    sl = encoders(1,1);
    sr = encoders(2,1);
  end;
  
else
  disp('Wrong input. Check your arguments');
end;