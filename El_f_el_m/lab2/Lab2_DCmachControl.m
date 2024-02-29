%**********************************************************************
% For current and speed control of separately excited DC-machine
%**********************************************************************
clear;          % clear workspace memory
close all;      % closing all plot windows
clc;            % clear command window

set(0,'defaulttextfontsize',12);
set(0,'defaultaxesfontsize',12);
set(0,'defaultaxesfontweight','bold');
set(0,'defaultlinelinewidth',2);

%**********************************************************************
% DC-machine Parameters
%**********************************************************************
% % Edit the following paramters:
Ra       = 1.9; % Armature resistance [Ohm]
La       = .01985; % Armature inductance [H]
K_lambda = 0.844; % Flux linkage constant [Wb/A]
B        = 0.004; % Viscous damping constant [Nms/rad]
J        = 0.054; % Total inertia [kgm2]
%**********************************************************************
% Do NOT edit the following parameters:
    ifeild = 1.5; % Field current [A]
    lambda = K_lambda*ifeild; % Flux linkage [Wb]
%**********************************************************************


%**********************************************************************
% Current Controller Parameters
%**********************************************************************
% Edit the following paramters:
tr_c    = 1;
tau_c   = 1;
alfa_c  = 1;
Kpc     = 2.24;
Kic     = 101.8;    
%**********************************************************************


%**********************************************************************
% Speed Controller Parameters
%**********************************************************************
% Edit the following paramters:
tr_w    = 1;
tau_w   = 1;
alfa_w  = 1;
Kpw     = 1;
Kiw     = 1;
%**********************************************************************



% % % **********************************************************************
%% Task 2.1 - Only machine - Load Steps
% % %----------------------------------------------------------------------
vT_steptime = 0.1;
vT_step = 180; 

% TL_steptime = 0;
% TL_step = 0;
TL_steptime1 = 1.1;
TL_steptime2 = 1.5;
TL_step = 30;

% % Call solver using panel settings, i.e Variable-step
% %----------------------------------------------------------------------
Tstart=0;       % Starting time for the simulation [s]
Tstop=2;      % End time for the simulation [s]
sim('Lab2_DCmach',[Tstart,Tstop]); 
% % % Ploting the simulation data
% % %----------------------------------------------------------------------
figure('Name','Loaded DC machine')
Tst = 1;
Tend = 2;
subplot(2,2,1)
hold on; grid on;
    plot(time,vT,time,vR,time,vL,time,ea)
    legend('v_T','v_R','v_L','e_a','Location','Best')
    xlabel('Time [s]')
    ylabel('Voltage [V]')
    xlim([Tst Tend])
subplot(2,2,2)
hold on; grid on;
    plot(time,Te,time,TLextra+B*wr)
    legend('T_d_e_v','T_L')
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([Tst Tend])
subplot(2,2,3)
hold on; grid on;
    plot(time,ia)
    legend('i_a')
    xlabel('Time [s]')
    ylabel('Current [A]')
    xlim([Tst Tend])
subplot(2,2,4)
hold on; grid on;
    plot(time,wr*30/pi)
    legend('w_r','Location','Best')
    xlabel('Time [s]')
    ylabel('Speed [RPM]')
    xlim([Tst Tend])
% print Loaded_DCmachine -dpdf% % % **********************************************************************

% % % **********************************************************************
%% Task 2.2 Current control
% % %----------------------------------------------------------------------
ia_steptime = 0;
ia_step = 0; 

TL_steptime1 = 0;
TL_steptime2 = 0;
TL_step = 0;

% % Call solver using panel settings, i.e Variable-step
% %----------------------------------------------------------------------
Tstart=0;       % Starting time for the simulation [s]
Tstop=1;      % End time for the simulation [s]
sim('Lab2_DCmach_CC',[Tstart,Tstop]); 

% % Ploting the simulation data
% % %----------------------------------------------------------------------
% figure('Name','Simulated Current Control of the DC machine')
%     subplot(2,2,1)
%     hold on; grid on;
%         plot(time,vT,'-',time,vR,'-',time,vL,'-',time,ea,'-',time,v_cp,'--',time,v_ci,'--',time,ea_hat,'c:'); legend('v_T','v_R','v_L','e_a','v_{c,p}','v_{c,i}','e_{a,hat}');
%         xlabel('Time [s]');
%         ylabel('Voltage [V]');
%     subplot(2,2,2)
%     hold on; grid on;
%         plot(time,Te,time,TLextra); legend('T_{e}','T_{L,extra}');
%         xlabel('Time [s]');
%         ylabel('Torque [Nm]');
%     subplot(2,2,3)
%     hold on; grid on;
%         plot(time,ia,time,ia_ref,'--'); legend('i_a','i_{a,ref}');
%         xlabel('Time [s]');
%         ylabel('Current [A]');
%     subplot(2,2,4)
%     hold on; grid on;
%         plot(time,wr*30/pi);
%         xlabel('Time [s]');
%         ylabel('Speed [RPM]');
% % % **********************************************************************
% 
% 
% 
% % % **********************************************************************
%% Task 2.3 Speed control
% % % **********************************************************************
% wrrefsteptime=0;
% wrrefstep=0;
% 
% TL_steptime1 = 0;
% TL_steptime2 = 0;
% TL_step = 0;
% 
% 
% % Call solver using panel settings, i.e Variable-step
% % %----------------------------------------------------------------------
% Tstart=0.5;       % Starting time for the simulation [s]
% Tstop=5;      % End time for the simulation [s]
% sim('Lab2_DCmach_CC_SC',[Tstart,Tstop]); 
%   
% % Ploting the simulation data
% % %----------------------------------------------------------------------
% figure('Name','Simulated Speed Control of the DC machine')
%     subplot(2,2,1)
%     hold on; grid on;
%         plot(time,vT,'-',time,ea,'-'); legend('v_T','e_a');
%         xlabel('Time [s]')
%         ylabel('Voltage [V]')
%     subplot(2,2,2)
%     hold on; grid on;
%         plot(time,Te,time,Te_ref,'--',time,TLextra); legend('T_{e}','T_{e,ref}','T_{L,extra}')
%         xlabel('Time [s]')
%         ylabel('Torque [Nm]')
%     subplot(2,2,3)
%     hold on; grid on;
%         plot(time,ia,time,ia_ref,'--'); legend('i_a','i_{a,ref}');
%         xlabel('Time [s]')
%         ylabel('Current [A]')
%     subplot(2,2,4)
%     hold on; grid on;
%         plot(time,wr*30/pi,time,wr_ref*30/pi,'--'); legend('w_r','w_{r,ref}','Location','Best');
%         xlabel('Time [s]')
%         ylabel('Speed [RPM]')
% 
% % % ----------------------------------------------------------------------
% % 
% % 
% % %----------------------------------------------------------------------
%% Task 3 - Plot measurements
% % %----------------------------------------------------------------------
% % 
% Filename_Measurements='SpeedControl_52rads';
% lokal = load(Filename_Measurements);   
% 
% t = lokal (1,:);            % time [s]
% ia = lokal(2,:);            % armature current [A]
% iaref = lokal(3,:);         % armature current reference [A]
% w = lokal(4,:);             % rotor speed [rad/s]
% ua = lokal(5,:);            % armature voltage [V]
% wref = lokal(6,:);          % rotor speed reference [rad/s]
% 
% 
% subplot(2,2,1)
%     plot(t,ua,'c--'); 
%     legend('Sim: v_T','Sim: e_a','Meas: v_T');
% 
% subplot(2,2,3)
%     plot(t,ia,'c:',t,iaref,'c--');
%     legend('Sim: i_a','Sim: i_{a,ref}','Meas: i_a','Meas: i_{a,ref}');
% 
% subplot(2,2,4)
%     plot(t,w*30/pi,'c:',t,wref*30/pi,'c--');
%     legend({'Sim: w_r','Sim: w_{r,ref}','Meas: w_r','Meas: w_{r,ref}'},'Location','Best');
% 
%  % % **********************************************************************
