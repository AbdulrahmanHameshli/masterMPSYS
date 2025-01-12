% [x,C,arcpts] = ododdforward(xs,Cs,dal,dar,b,rl,rr,kl,kr)
%   Simulate differential drive odometry one step forward. Unlike the old
%   odoslsrforward, takes the angular increments and the wheel radii as
%   inputs (as a generalization for AKF localization) and uses consistent
%   naming.
%
% v.1.0, koa, Jan 2015, anronaut autocalibration project

function [x,C,arcpts] = ododdforward(xs,Cs,dal,dar,b,rl,rr,kl,kr)

NARCPOINTS = 30;

sl = dal * rl;  
sr = dar * rr;

Fx = eye(3);
Fu = zeros(3,2);
U = diag([kl*abs(sl),kr*abs(sr)]);

s = (sr + sl) / 2;
dtheta = (sr - sl) / b;
if abs(sr - sl) > 1.0e-10
  alpha = xs(3) + dtheta;
  % Forward model (first moments, this is the no-approximation model variant)
  xf = xs(1) + (b*(sl+sr))/(2*(sr-sl))*(-sin(xs(3))+sin(alpha));
  yf = xs(2) + (b*(sl+sr))/(2*(sr-sl))*( cos(xs(3))-cos(alpha));
  r = s / dtheta;
  % Generate arc points
  psi = xs(3) + pi/2;
  xc  = xs(1) + r*cos(psi);
  yc  = xs(2) + r*sin(psi);
  phi1 = psi+pi;
  phi2 = phi1 + dtheta;
  t = phi1:(phi2-phi1)/NARCPOINTS:phi2;
  arcpts = [r*cos(t); r*sin(t)] + [xc; yc]*ones(1,length(t));
  % Second moments
  Fx(1,3) = r*(-cos(xs(3)) + cos(alpha));
  Fx(2,3) = r*(-sin(xs(3)) + sin(alpha));
  ssinv = b / (sr-sl)^2;
  Fu(1,1) = -(sr*ssinv)*(sin(xs(3)) - sin(alpha)) - r/b*cos(alpha);
  Fu(1,2) =  (sl*ssinv)*(sin(xs(3)) - sin(alpha)) + r/b*cos(alpha);
  Fu(2,1) =  (sr*ssinv)*(cos(xs(3)) - cos(alpha)) - r/b*sin(alpha);
  Fu(2,2) = -(sl*ssinv)*(cos(xs(3)) - cos(alpha)) + r/b*sin(alpha);
else
  beta = xs(3) + dtheta/2;
  % Forward model (first moments)
  xf = xs(1) + s*cos(beta);
  yf = xs(2) + s*sin(beta);
  % Generate arc points
  arcpts = [xs(1) xf; xs(2) yf];
  % Second moments
  Fx(1,3) = -s*sin(beta);
  Fx(2,3) =  s*cos(beta);
  Fu(1,1) = 1/2*cos(beta) + s/(2*b)*sin(beta);
  Fu(1,2) = 1/2*cos(beta) - s/(2*b)*sin(beta);
  Fu(2,1) = 1/2*sin(beta) - s/(2*b)*cos(beta);
  Fu(2,2) = 1/2*sin(beta) + s/(2*b)*cos(beta);
end
Fu(3,1) = -1/b;
Fu(3,2) =  1/b;
tf = xs(3) + dtheta;

x = [xf; yf; tf];
C = Fx*Cs*Fx' + Fu*U*Fu';
