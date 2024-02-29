%**********************************************************************
% For simulating direct start of separately excited DC-machine and plot
% measured signals
%**********************************************************************

clc;            % clear command window
clear;          % clear workspace memory
close all;      % closing all plot windows

set(0,'defaulttextfontsize',14);
set(0,'defaultaxesfontsize',14);
set(0,'defaultaxesfontweight','bold');
set(0,'defaultlinelinewidth',2);


% Parameter definitions
%**********************************************************************
% Edit the following paramters:
Ra       = 1; % Armature resistance [Ohm]
La       = 1; % Armature inductance [H]
K_lambda = 1; % Flux linkage constant [Wb/A]
B        = 1; % Viscous damping constant [Nms/rad]
J        = 1; % Total inertia [kgm2]
%----------------------------------------------------------------------

% Pre-set parameters:
ifeild = 1.5; % Field current [A]
lambda = K_lambda*ifeild; % Flux linkage [Wb]
%**********************************************************************

% Load the measurements of the direct start
%**********************************************************************
lokal = load('DirectStart.txt');

time_mes = lokal(1,:); % time [s]
ia_mes = lokal(2,:);   % armature current [A]
wr_mes = lokal(3,:);   % rotor speed [rad/s]
vT_mes = lokal(4,:);   % terminal voltage [rad/s]
if_mes = lokal(5,:);   % Feild current [A]

% Finding the time for the terminal voltage step
num1 = find(vT_mes > 20);
% Finding the time for when the DC machine is in steady state
num2 = find(time_mes > (time_mes(num1(1))+1.1));

% Setting the parameters of the step functions
%**********************************************************************
vT_steptime = time_mes(num1(1));
vT_step = mean(vT_mes(num2)); % Calculating the voltage from the measurement
ifeild = mean(if_mes(num2)); % Calculating the current from the measurement
lambda = K_lambda*ifeild;

% Call solver using panel settings, i.e Variable-step
%**********************************************************************
Tstart=0;       % Starting time for the simulation [s]
Tstop=3;      % End time for the simulation [s]
sim('DCpanel',[Tstart,Tstop]); 

% Ploting the simulation and measurement data
%**********************************************************************
figure('Name','Direct start of the DC machine')
Tend = 2;
subplot(2,2,1)
hold on; grid on;
    plot(time_mes,vT_mes,time,vT,time,vR,time,vL,time,ea)
    legend('v_T_,_m_e_s','v_T','v_R','v_L','e_a','Location','Best')
    xlabel('Time [s]')
    ylabel('Voltage [V]')
    xlim([0 Tend])
subplot(2,2,3)
hold on; grid on;
    plot(time_mes,ia_mes,time,ia,time_mes,if_mes*10)
    legend('i_a_,_m_e_s','i_a','10*i_f_,_m_e_s')
    xlabel('Time [s]')
    ylabel('Current [A]')
    xlim([0 Tend])
subplot(2,2,2)
plot(time,Tdev,time,TLextra+B*wr)
hold on; grid on;
    legend('T_d_e_v','T_L')
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([0 Tend])
subplot(2,2,4)
hold on; grid on;
    plot(time_mes,wr_mes*30/pi,time,wr*30/pi)
    legend('w_r_,_m_e_s','w_r','Location','Best')
    xlabel('Time [s]')
    ylabel('Speed [RPM]')
    xlim([0 Tend])
% print DirectStart -dpdf

