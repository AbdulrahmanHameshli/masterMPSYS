% Function that plots the robot and its observations into the current figure
% and returns their handles

function [hrob, hobs] = drawrobot(xrobot, observations)

xr = xrobot(1);
yr = xrobot(2);

% Plot robot
%hrob = rectangle('Position',[robot.xgt(1)-0.5 robot.xgt(2)-0.5 1 1],...
%  'Curvature',[0.7 0.7],'FaceColor','r','EdgeColor','m');  % Octave had issues with this cmd
hrob = fill([xr-0.5 xr+0.5 xr+0.5 xr-0.5],[yr-0.5 yr-0.5 yr+0.5 yr+0.5],'r');

% Plot observations
hobs = zeros(1,4);
if observations(1)  % N
  hobs(1) = fill(xr+[-0.5 0.5 0.5 -0.5], yr+[0.5 0.5 1 1],'b','EdgeColor','none');
end
if observations(2)  % S
  hobs(2) = fill(xr+[-0.5 0.5 0.5 -0.5], yr+[-0.5 -0.5 -1 -1],'b','EdgeColor','none');
end
if observations(3)  % W
  hobs(3) = fill(xr+[-1 -0.5 -0.5 -1], yr+[-0.5 -0.5 0.5 0.5],'b','EdgeColor','none');
end
if observations(4)  % E
  hobs(4) = fill(xr+[0.5 1 1 0.5], yr+[-0.5 -0.5 0.5 0.5],'b','EdgeColor','none');
end
% Remove empty elements so that h contains only actual number of handles
hobs = hobs(hobs~=0);
