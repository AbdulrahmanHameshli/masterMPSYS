% Exercise 03 - Wheeled Robot Kinematics


%% Prepare Session
clear all 
clc
addpath(genpath('utils'))
init_wheels

%% Task 3

clc
[J1,J2,C1,C2,dm,ds,~] = computeConstraints(w1);
disp("wheel set 1")
disp("type:      ("+dm+","+ds+")")
disp("should be: (1,1)");

[J1,J2,C1,C2,dm,ds,~] = computeConstraints(w1);
disp("wheel set 1")
disp("type:      ("+dm+","+ds+")")
disp("should be: (1,1)");
[J1,J2,C1,C2,dm,ds,~] = computeConstraints(w2);
disp("wheel set 2")
disp("type:      ("+dm+","+ds+")")
disp("should be: (1,2)");
[J1,J2,C1,C2,dm,ds,~] = computeConstraints(w3);
disp("wheel set 3")
disp("type:      ("+dm+","+ds+")")
disp("should be: (2,0)");
[J1,J2,C1,C2,dm,ds,~] = computeConstraints(w4);
disp("wheel set 4")
disp("type:      ("+dm+","+ds+")")
disp("should be: (3,0)");
[J1,J2,C1,C2,dm,ds,~] = computeConstraints(w5);
disp("wheel set 5")
disp("type:      ("+dm+","+ds+")")
disp("should be: (2,1)");

%% Task 4.a (see pdf)
clear all
addpath(genpath('utils'))
init_wheels
% Wheel setup
w41.type = 0;
w41.params = [wheelr, wheelw];
w42 = w41;
w43 = w41;

w41.pose = [0, -1, 0];     
w42.pose = [0, 1, 0];   
w43.pose = [5,0, pi/4]; 
wheel_poses = [w41.pose; w42.pose; w43.pose];

plot_wheel(wheel_poses );


orientation_variation = linspace(0,0.1, 100); % Small increments
singular_values = zeros(size(orientation_variation));

% Loop over the orientation variations
for i = 1:length(orientation_variation)
    w43.pose(3) = w43.pose(3) + orientation_variation(i);   
    [~, ~,~ , ~, ~, ~,C1Star] = computeConstraints([w41, w42, w43]);
    singular_values(i) = min(svd(C1Star));
    pos(i) = w43.pose(3); 
end

% Plot the singular values as a function of orientation variation
figure;
plot(rad2deg(pos), singular_values, 'LineWidth', 1.5);
xlabel('Orientation Variation of w41 (degree)');
ylabel('Smallest Singular Value of C1');
title('Effect of Orientation Variation on Smallest Singular Value');
grid on;





%% Task 4.b

f2 = figure(); hold on; axis([-4 4 -4 4]); axis equal
title("ICR test:")

ICR = getICR([0;0;pi/3], w1);
ICR_gt = [0.3031;0.5250];
drawrobot([0;0;pi/3], [0,0,0], c1, w1);
plot(ICR(1), ICR(2), "ro");
plot(ICR_gt(1), ICR_gt(2), "b+");
disp("ICR is ")
ICR
disp("Should be "+ ICR_gt(1)+ " "+ ICR_gt(2)+".");
% 
ICR = getICR([2;0;pi/3], w2);
ICR_gt = [1.7000;-0.5196];
drawrobot([2;0;pi/3], [0,0,0], c2, w2);
plot(ICR(1), ICR(2), "ro");
plot(ICR_gt(1), ICR_gt(2), "b+");
disp("ICR is ")
ICR
disp("Should be "+ ICR_gt(1)+ " "+ ICR_gt(2)+".");

w22 = w2;
w22(1).pose(3) = pi/4;
ICR = getICR([-2;0;pi/3], w22);
ICR_gt = [-1.8117;0.9364];
drawrobot([-2;0;pi/3], [0,0,0], c2, w22);
plot(ICR(1), ICR(2), "ro");
plot(ICR_gt(1), ICR_gt(2), "b+");
disp("ICR is ")
ICR
disp("Should be "+ ICR_gt(1)+ " "+ ICR_gt(2)+".");

ICR = getICR([-2;-3;pi/3], w3);
ICR_gt = [inf;inf];
drawrobot([-2;-3;pi/3], [0,0,0], c3, w3);
plot(ICR(1), ICR(2), "ro");
plot(ICR_gt(1), ICR_gt(2), "b+");
disp("ICR is ")
ICR
disp("Should be "+ ICR_gt(1)+ " "+ ICR_gt(2)+".");

ICR = getICR([0;-3;pi/3], w7);
ICR_gt = [nan; nan];
drawrobot([0;-3;pi/3], [0,0,0], c7, w7);
plot(ICR(1), ICR(2), "ro");
plot(ICR_gt(1), ICR_gt(2), "b+");
% concatenating strings an nan prints <missing>
disp("Non existing ICR:")
ICR
disp("Should be NaN NaN")



% pause()
% close(f2)



%% Simulation playground (computeConstraints.m must be implemented)
use_keyboard = true;
simRobot(w1, c1, 0.01, use_keyboard);
simRobot(w2, c2, 0.01, use_keyboard);
simRobot(w3, c3, 0.01, use_keyboard);
simRobot(w4, c4, 0.01, use_keyboard);
simRobot(w5, c5, 0.01, use_keyboard);

close all


