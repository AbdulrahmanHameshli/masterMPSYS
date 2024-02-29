%**********************************************************************
% For simulating direct start of separately excited DC-machine
%**********************************************************************
clear;          % clear workspace memory
close all;      % closing all plot windows
clc;            % clear command window

set(0,'defaulttextfontsize',14);
set(0,'defaultaxesfontsize',14);
set(0,'defaultaxesfontweight','bold');
set(0,'defaultlinelinewidth',2);

% Parameter definitions
%**********************************************************************
% Edit the following paramters:
Ra       = 1.9; % Armature resistance [Ohm]
La       = 19.85; % Armature inductance [H]
K_lambda = 0.844; % Flux linkage constant [Wb/A]
B        = 0.004; % Viscous damping constant [Nms/rad]
%----------------------------------------------------------------------

% Pre-set parameters:
J        = 0.05; % Total inertia [kgm2]
ifeild = 1.5 * 0.9; % Field current [A]
lambda = K_lambda*ifeild; % Flux linkage [Wb]


% Simulation variables
%**********************************************************************
vT_steptime = 0.1;
vT_step = 180; 

TL_steptime1 = 0;
TL_steptime2 = 0;
TL_step = 0;

% Call solver using panel settings, i.e Variable-step
%**********************************************************************
Tstart=0;       % Starting time for the simulation [s]
Tstop=0.8;      % End time for the simulation [s]
sim('DCpanel',[Tstart,Tstop]); 

% Ploting the simulation and measurement data
%**********************************************************************
figure('Name','Direct start of the DC machine')
subplot(2,2,1)
hold on; grid on;
    plot(time,vT,time,vR,time,vL,time,ea);
    legend('v_T','v_R','v_L','e_a','Location','Best')
    xlabel('Time [s]')
    ylabel('Voltage [V]')
subplot(2,2,3)
hold on; grid on;
    plot(time,ia)
    legend('i_a')
    xlabel('Time [s]')
    ylabel('Current [A]')
    ylim([0 100]);
subplot(2,2,2)
hold on; grid on;
    plot(time,Tdev,time,TLextra+B*wr)
    legend('T_d_e_v','T_L')
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    ylim([0 100]);
subplot(2,2,4)
hold on; grid on;
    plot(time,wr*30/pi);
    legend('w_r','Location','Best')
    xlabel('Time [s]')
    ylabel('Speed [RPM]')
    ylim([0 1600]);
%     print DirectStart -dpdf

