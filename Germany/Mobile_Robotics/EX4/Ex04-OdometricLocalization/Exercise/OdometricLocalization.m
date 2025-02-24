% Mobile Robotics Exercise
% Uncertainty Propagation Odometry

% Main Script
% Script that compares and plots three ways to do error propagation across
% the sl,sr-wheel space odometry model. The methods are First-Order,
% Monte Carlo and Unscented Transform.
% Starting from a given start pose and a number of test positions
% for which the sl,sr displacements are given the data is fed into the
% functions for the propagation model that calculates the final poses and their
% covariance estimates.

close all;
clear all;
clc;

addpath(genpath("lib/"))

%% Define robot and start pose and errors
b = 0.32; kL = 0.002; kR = kL;
nSamples = 100;
poseStart = [0.4; 0.4; 20*pi/180];
R = [cos(poseStart(3)) -sin(poseStart(3)) 0; sin(poseStart(3)) cos(poseStart(3)) 0; 0 0 1];
% Start covariance matrix for the pose
covStart = R*diag([0.001 0.003 0.001])*R';

% Define test final positions (for plotting only)
testFinalPositions = [-0.5742, 0.0579, 1.0747, 1.9974, 1.1434;
               1.3224, 1.3396, 1.1776, 0.9814,-0.1807];
% Path lengths travelled by left and right wheel to reach test poses
sL = [2.4006, 1.0681, 0.9127, 1.7000, 1.4499];
sR = [3.7026, 2.0734, 1.2373, 1.7000, 0.8021];

nTestPoses = length(testFinalPositions);

%% Initialize data structure to store all relevant information
data.poseStart = poseStart;
data.covStart = covStart;
for i = 1:nTestPoses
    % Define system state [xPos, yPos, theta, sL, sR] as input mean
    data.testPoses(i).x = [poseStart; sL(i); sR(i)];

    % Input covariance matrix (for sL and sR)
    covU = zeros(2,2);
    covU(1,1) = kL*abs(sL(i));
    covU(2,2) = kR*abs(sR(i));
    % Define C (overall covariance matrix) as the blockdiagonal matrix
    % aligned to the state
    data.testPoses(i).C = blkdiag(covStart, covU);
end

%% Plot start pose and uncertainty and test final positions
figure(1); clf; hold on; box on; axis equal;
drawrobot(data.poseStart,'k',2,b,b);
drawprobellipse(data.poseStart,data.covStart,0.99,'b');
plot(testFinalPositions(1,:),testFinalPositions(2,:),'+','Color',[.6 .6 .6],'LineWidth',2,'MarkerSize',15) 

%% First-Order approach: generate forward solution and propagate errors
disp("Propagating: First Order")
for i = 1:nTestPoses
    [poseOut, covOut, arcPoints] = propagatefo(data.testPoses(i).x, data.testPoses(i).C,b);
    data.testPoses(i).poseFO = poseOut;
    data.testPoses(i).covFO = covOut;
    data.testPoses(i).arcPoints = arcPoints;
end

%% Monte Carlo approach: generate and transfer samples
disp("Propagating: Monte Carlo")
for i = 1:nTestPoses
    [poseOut, covOut, samples] = ...
        propagatemc(nSamples,data.testPoses(i).x, data.testPoses(i).C,b);
    data.testPoses(i).poseMC = poseOut;
    data.testPoses(i).covMC = covOut;
    data.testPoses(i).samples = samples;
end

%% Unscented Transform approach: compute and transfer sigma points
disp("Propagating: Unscented Transform")
for i = 1:nTestPoses
    [poseOut, covOut] = ...
        propagateut(data.testPoses(i).x, data.testPoses(i).C,b);
    data.testPoses(i).poseUT = poseOut;
    data.testPoses(i).covUT = covOut;
end

%% Plot final poses, arcs and samples
disp("Populating Plot")
for i = 1:nTestPoses
    plot(data.testPoses(i).arcPoints(1,:),data.testPoses(i).arcPoints(2,:),'k-');
    drawrobot(data.testPoses(i).poseFO,[0 0 .5],2,b,b);
    drawprobellipse(data.testPoses(i).poseFO,data.testPoses(i).covFO,0.99,[0 0 .5]);
  
    for j = 1:size(data.testPoses(i).samples,2)
        drawreference(data.testPoses(i).samples(:,j),'',0.03,[.6 .6 .6]);
    end
    drawrobot(data.testPoses(i).poseMC,[.7 .7 .7],2,b,b);
    drawprobellipse(data.testPoses(i).poseMC,data.testPoses(i).covMC,0.99,[.7 .7 .7]);

    drawrobot(data.testPoses(i).poseUT,[.5 0 0],2,b,b);
    drawprobellipse(data.testPoses(i).poseUT,data.testPoses(i).covUT,0.99,[.7 0 0]);  
end

title('blue: 1st order / red: unscented / gray: MC');

%% Calibrating the growth coefficient
% this task is implemented in calibK.m
% pause() % pause to not take over the previous plot
close all;
nRuns = 10;
visualize = true;

% initial growth coefficient approximation, it is an underapproximation
% because it can only grow
kInit = 1e-6;

calibK(kInit, nRuns, visualize);
