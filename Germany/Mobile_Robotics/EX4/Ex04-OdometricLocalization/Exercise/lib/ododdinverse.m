% ododdinverse: inverse kinematics for differential drive kinematic configuration
%   [sl,sr] = ododdinverse(xs,xe,b)
%
% v.1.0, koa, Evolution Robotics Inc.


function [sl,sr] = ododdinverse(xs,xe,b)

dx = xe(1) - xs(1);
dy = xe(2) - xs(2);
d  = sqrt(dx^2 + dy^2);
dtheta = 2*diffangle(atan2(dy,dx),xs(3));

if abs(diffangle(dtheta,0.0)) > 1.0e-10,
  r = d/(2*sin(dtheta/2));
  sl = dtheta*(r - b/2);
  sr = dtheta*(r + b/2);
else
  sl = d;
  sr = d;
end;