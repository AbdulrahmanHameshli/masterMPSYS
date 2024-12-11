% Script TestPosController.m
% v.1.0, 13.07.03, koa: Creation date
% v.1.1, 16.08.03, koa: Adaptations for v.1.2 of PositionController
%                       and v.1.1.1 of PosControlStep
% v.1.2, August 2013, koa: adaptations to librobotics, renamings

% Init Session
clear;
addpath('~/SRL/Research/lib/librobotics');

% Define start and goal poses
xstart = [0; 0; 2*pi/9];
xgoal  = [2; 1.5; -6*pi/5];

deltaT = 0.02;

wheelbase = 0.4;
dir = 1;

% Call controller
[xvec, vvec] = positioncontrol(xstart, xgoal, dir, deltaT, wheelbase);

% Plot result
figure(1); clf; hold on; box on;
xlabel('x [mm]'); ylabel('y [mm]'); title('Robot trajectory');
n = size(xvec,2);
for i = 1:n,
  drawrobot(xvec(:,i), [i/(2*n)+0.5 0.9 1-i/(2*n)], 1);
end;
drawrobot(xstart,'k',1);		% plot start pose
drawrobot(xgoal, 'k',1);    % plot goal pose
av = axis; axis([av(1)-0.05 av(2)+0.05 av(3)-0.05 av(4)+0.05]);
axis equal; 

figure(2); clf; hold on; box on; grid on;
plot(1:size(vvec,2), vvec(1,:), 'b', 1:size(vvec,2), vvec(2,:), 'r');
xlabel('time index'); ylabel('v [m/s]'); title('Wheel speeds (vl, vr)');
axis(1.2*axis); legend('vl','vr');
