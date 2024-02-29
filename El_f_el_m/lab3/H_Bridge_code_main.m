
close all
clear
clc
tic
%% Parameters

% Machine
EM.R_a    = 1.9;         % Armature resistance [Ohm]
EM.L_a    = 19.86e-3;    % Armature inductance [H]
EM.lambda = 0.844*1.5;   % Flux linkage [Wb]
EM.B      = 0.004;       % Viscous damping constant [Nms/rad]
EM.J      = 0.055;       % Total inertia [kgm2]

% Current Controller
ctrl.tri     = 20e-3;
ctrl.tau_c   = ctrl.tri/log(9);
ctrl.alpha_c = 1/ctrl.tau_c;
ctrl.Kpc     = ctrl.alpha_c*EM.L_a;
ctrl.Kic     = ctrl.alpha_c*EM.R_a;
ctrl.lambda  = EM.lambda;

% Speed Controller
ctrl.trw     = 10*ctrl.tri;
ctrl.tau_w   = ctrl.trw/log(9);
ctrl.alpha_w = 1/ctrl.tau_w;
ctrl.Kpw     = ctrl.alpha_w*EM.J;
ctrl.Kiw     = ctrl.alpha_w*EM.B;
% 


% Step - Speed Reference
omega_ref.time_1 = 0.5;
omega_ref.step_1 = +1000 * pi / 30; % rad/s
omega_ref.time_2 = 3.0;
omega_ref.step_2 = -1000 * pi / 30; % rad/s
omega_ref.time_3 = 4;
omega_ref.step_3 = -1000 * pi / 30; % rad/s
omega_ref.time_4 = 6.5;
omega_ref.step_4 = +1000 * pi / 30; % rad/s

% Step - Load Torque
torque_load.time_1 = 1.5;
torque_load.step_1 = +10; % N*m
torque_load.time_2 = 2.5;
torque_load.step_2 = -10; % N*m
torque_load.time_3 = 5.0;
torque_load.step_3 = -10; % N*m
torque_load.time_4 = 6.0;
torque_load.step_4 = +10; % N*m


% Circuit
DC.U = 260;
MOSFET.Ron = 1e-3;
MOSFET.Rd  = 1e-1;
MOSFET.Vf  = 0e-3;
MOSFET.Rs  = 1e+5;
MOSFET.Cs  = 1e+5;
f.sw = 20 * 1e3;
T.sw = 1 / f.sw;
T.FFT = T.sw / 10;

%%


sim('H_Bridge_Converter_and_DC_Machine.slx',[0,8])
toc