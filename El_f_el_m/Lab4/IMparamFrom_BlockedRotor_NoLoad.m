
% MATLAB Code - IM Parameter Post Process Code

% EEN155 Electric Drives and fields
% Division of Electric Power Engineering
% Chalmers University of Technology 

% Version 1.0    2018-10-08    Junfei Tang <junfei.tang@chalmers.se>
% Version 1.1    2018-10-11    Junfei Tang <junfei.tang@chalmers.se>
% Version 1.2    2018-10-12    Junfei Tang <junfei.tang@chalmers.se>
% Version 1.3    2022-02-28    Emma Grunditz <emma.grunditz@chalmers.se>
% Version 1.4    2023-02-02    Emma Grunditz <emma.grunditz@chalmers.se>


% The equivalent curcuit of the induction machine, including the core loss
% resistance is shown below

%     I1
%   o-->---R1---X1----------X'2---R'2-----
%  +               |   |                 |
%  U1             Rfe  Xm                R'2*(1-s)/s
%  -               |   |                 |
%   o-------------------------------------


% Notes:
%
% (1) Three tests are performed: Test 1,2 and 3. The results are presented
% in the respective secions below. 
%
% (2) In the 'EDIT HERE - Parameter Calculation' Section; you should fill in the
% formulas required to calculate the induction machine parameters.


%%
close all
clear
clc

figure_configuration_code;

% Signal scaling
K_voltage        =  650 / 5;
K_current        =   35 / 5;
K_speed          = 1500 / 150;

%% Test 1 - Stator winding dc resistance - To get R_1

%     I1=I_dc
%   o-->---R1----
%  +            |
%  U1=V_dc      |
%  -            |
%   o------------


% The following results were achieved when measuring the stator phase
% winding resistance

V_dc=3.8; % (V) Measured dc-voltage over one stator phase winding
I_dc=3;   % (A) Measured dc-current in stator phase winding

%% Test 2 - Locked Rotor Test - To get R_1, R_2 and L_1, L_2
% w_r=0  =>   s=1  =>   R'2*(1-s)/s = 0

% Rfe >> R2
% Xm >> X2
% => The equivalent curcuit can approximately be reduced to:

%     I1
%   o-->---R1---X1----------X'2---R'2-----
%  +                                     |
%  U1                                    |
%  -                                     |
%   o-------------------------------------
%
% 
% p(t) = U1*I1 + U2*I2 + U3*I3
% P_average = average ( p(t) )
% P_input = P_average  =  P_output = P_(R1+R2)

% q(t) = 1/sqrt(3) * (  I1*(U2-U3) + I2*(U3-U1) + I3*(U1-U2)  )
% Q_average = average ( q(t) )
% Q_input = Q_average  =  Q_output  = Q_(X1+X'2)
% Assume X1 = X'2


% -- Load Measured Data -- %

data = load('Test_2_IM_Locked_Rotor');

u1 = data(:,5) * K_voltage;
u2 = data(:,6) * K_voltage;
u3 = data(:,7) * K_voltage;
i1 = data(:,1) * K_current;
i2 = data(:,2) * K_current;
i3 = data(:,3) * K_current;
n  = data(:,8) * K_speed;

time = (0:1:(length(n)-1))/5e3;

% -- Process Data -- %

% current rms
u_lockedrotor = [u1,u2,u3];
i_lockedrotor = [i1,i2,i3];

U_rms_lockedrotor = mean(rms(u_lockedrotor));
I_rms_lockedrotor = mean(rms(i_lockedrotor));

% active and reactive power
p_lockedrotor = i1 .* u1 + i2 .* u2 + i3 .* u3;
q_lockedrotor = (i1.*(u2-u3) + i2.*(u3-u1) + i3.*(u1-u2)) / sqrt(3);

P_lockedrotor = mean(p_lockedrotor);
Q_lockedrotor = mean(q_lockedrotor);

% -- Plot -- %

% plot the whole time series
figure('Name','Locked Rotor - whole time series')
subplot(2,2,1)
hold on, grid on;
    plot(time,u1,'Color',color_2014b_blue)
    plot(time,u2,'Color',color_2014b_orange)
    plot(time,u3,'Color',color_2014b_green)
    legend('{\itu}_{s.1}','{\itu}_{s.2}','{\itu}_{s.3}')
    xlabel('Time [s]')
    ylabel('Stator Voltage (V)')
subplot(2,2,2)
    hold on, grid on;
    plot(time,i1,'Color',color_2014b_blue)
    plot(time,i2,'Color',color_2014b_orange)
    plot(time,i3,'Color',color_2014b_green)
    legend('{\iti}_{s.1}','{\iti}_{s.2}','{\iti}_{s.3}')
    xlabel('Time [s]')
    ylabel('Stator Current [A]')
subplot(2,2,3)
    hold on, grid on;
    plot(time,n,'Color',color_2014b_blue)
    legend('{\it\Omega}_{r}')
    xlabel('Time [s]')
    ylabel('Rotor Speed [rpm]')
subplot(2,2,4)
    hold on, grid on;
    plot(time,p_lockedrotor/1e3,'Color',color_2014b_blue)
    plot(time,q_lockedrotor/1e3,'Color',color_2014b_orange)
    plot(time,P_lockedrotor*ones(1,length(time))/1e3,'Color',color_2014b_blue,'LineStyle','--')
    plot(time,Q_lockedrotor*ones(1,length(time))/1e3,'Color',color_2014b_orange,'LineStyle','--')
    legend('{\itp}_{s}','{\itq}_{s}','{\itP}_{s}','{\itQ}_{s}')
    xlabel('Time [s]')
    ylabel('Power [kW] [kVar]')



% plot just a short part of the time-series

f_grid = 50;            % [Hz]
T_grid = 1 / f_grid;    % [s]
temp = time > time(end) - 2.5 * T_grid;

figure('Name','Locked Rotor - time zoomed in')
subplot(2,2,1)
    hold on, grid on;
    plot(time(temp),u1(temp),'Color',color_2014b_blue)
    plot(time(temp),u2(temp),'Color',color_2014b_orange)
    plot(time(temp),u3(temp),'Color',color_2014b_green)
    xlabel('Time [s]')
    ylabel('Stator Voltage (V)')
    legend('{\itu}_{s.1}','{\itu}_{s.2}','{\itu}_{s.3}')
subplot(2,2,2)
    hold on, grid on;
    plot(time(temp),i1(temp),'Color',color_2014b_blue)
    plot(time(temp),i2(temp),'Color',color_2014b_orange)
    plot(time(temp),i3(temp),'Color',color_2014b_green)
    xlabel('Time [s]')
    ylabel('Stator Current [A]')
    legend('{\iti}_{s.1}','{\iti}_{s.2}','{\iti}_{s.3}')
subplot(2,2,3)
    hold on, grid on;
    plot(time(temp),n(temp),'Color',color_2014b_blue)
    xlabel('Time [s]')
    ylabel('Rotor Speed [rpm]')
    legend('{\it\Omega}_{r}')
subplot(2,2,4)
    hold on, grid on;
    plot(time(temp),p_lockedrotor(temp)/1e3,'Color',color_2014b_blue)
    plot(time(temp),q_lockedrotor(temp)/1e3,'Color',color_2014b_orange)
    plot(time(temp),P_lockedrotor*ones(1,length(time(temp)))/1e3,'Color',color_2014b_blue,'LineStyle','--')
    plot(time(temp),Q_lockedrotor*ones(1,length(time(temp)))/1e3,'Color',color_2014b_orange,'LineStyle','--')
    xlabel('Time [s]')
    ylabel('Power [kW] [kVar]')
    legend('{\itp}_{s}','{\itq}_{s}','{\itP}_{s}','{\itQ}_{s}')

result_lockedrotor = {
                    'U_rms_lockedrotor' , U_rms_lockedrotor , 'V';   
                    'I_rms_lockedrotor' , I_rms_lockedrotor , 'A'; 
                    'P_lockedrotor'     , P_lockedrotor/1e3 , 'kW';    
                    'Q_lockedrotor'     , Q_lockedrotor/1e3 , 'kVar';     
                };

disp(result_lockedrotor);

%% Test 3 - No Load Test - to Get R_c and L_m
% w_r=w_s  =>   s=0  =>   R'2*(1-s)/s = infinite

% Rfe >> R1
% Xm >> X1
% => The equivalent curcuit can approximately be reduced to:

%     I1
%   o-->----------
%  +         |   |
%  U1        Rfe  Xm
%  -         |   |
%   o-------------
%
% 
% p(t) = U1*I1 + U2*I2 + U3*I3
% P_average = average ( p(t) )
% P_input = P_average  =  P_output = P_Rfe

% q(t) = 1/sqrt(3) * (  I1*(U2-U3) + I2*(U3-U1) + I3*(U1-U2)  )
% Q_average = average ( q(t) )
% Q_input = Q_average  =  Q_output  = Q_Xm

% -- Load Measured Data -- %

data = load('Test_3_IM_No_Load');

u1 = data(:,5) * K_voltage;
u2 = data(:,6) * K_voltage;
u3 = data(:,7) * K_voltage;
i1 = data(:,1) * K_current;
i2 = data(:,2) * K_current;
i3 = data(:,3) * K_current;
n  = data(:,8) * K_speed;

time = (0:1:(length(n)-1))/5e3;

% For t > 0.4s the mashine is in steady-state no-load operation
temp = time > 0.4;
Speedfakt = 1499/mean(n(temp));
n = n * Speedfakt;

% -- Process Data -- %

% current rms
u_noload = [u1,u2,u3];
i_noload = [i1,i2,i3];

U_rms_noload = mean(rms(u_noload));
I_rms_noload = mean(rms(i_noload));

% active and reactive power
p_noload = i1 .* u1 + i2 .* u2 + i3 .* u3;
q_noload = (i1.*(u2-u3) + i2.*(u3-u1) + i3.*(u1-u2)) / sqrt(3);

P_noload = mean(p_noload);
Q_noload = mean(q_noload);

% -- Plot -- %

% plot the whole time series
figure('Name','No Load - whole time series')
clf;
subplot(2,2,1)
    hold on, grid on;
    plot(time,u1,'Color',color_2014b_blue)
    plot(time,u2,'Color',color_2014b_orange)
    plot(time,u3,'Color',color_2014b_green)
    xlabel('Time [s]')
    ylabel('Stator Voltage (V)')
    legend('{\itu}_{s.1}','{\itu}_{s.2}','{\itu}_{s.3}')
subplot(2,2,2)
    hold on, grid on;
    plot(time,i1,'Color',color_2014b_blue)
    plot(time,i2,'Color',color_2014b_orange)
    plot(time,i3,'Color',color_2014b_green)
    xlabel('Time [s]')
    ylabel('Stator Current [A]')
    legend('{\iti}_{s.1}','{\iti}_{s.2}','{\iti}_{s.3}')
subplot(2,2,3)
    hold on, grid on;
    plot(time,n,'Color',color_2014b_blue)
    xlabel('Time [s]')
    ylabel('Rotor Speed [rpm]')
    legend('{\it\Omega}_{r}')
subplot(2,2,4)
    hold on, grid on;
    plot(time,p_noload/1e3,'Color',color_2014b_blue)
    plot(time,q_noload/1e3,'Color',color_2014b_orange)
    plot(time,P_noload*ones(1,length(time))/1e3,'Color',color_2014b_blue,'LineStyle','--')
    plot(time,Q_noload*ones(1,length(time))/1e3,'Color',color_2014b_orange,'LineStyle','--')
    xlabel('Time [s]')
    ylabel('Power [kW] [kVar]')
    legend('{\itp}_{s}','{\itq}_{s}','{\itP}_{s}','{\itQ}_{s}')



% plot the just a short part of the time-series

f_grid = 50;            % [Hz]
T_grid = 1 / f_grid;    % [s]
temp = time > time(end) - 2.5 * T_grid;

figure('Name','No Load - time zoomed in')
clf;
subplot(2,2,1)
    hold on, grid on;
    plot(time(temp),u1(temp),'Color',color_2014b_blue)
    plot(time(temp),u2(temp),'Color',color_2014b_orange)
    plot(time(temp),u3(temp),'Color',color_2014b_green)
    xlabel('Time [s]')
    ylabel('Stator Voltage (V)')
    legend('{\itu}_{s.1}','{\itu}_{s.2}','{\itu}_{s.3}')
subplot(2,2,2)
    hold on, grid on;
    plot(time(temp),i1(temp),'Color',color_2014b_blue)
    plot(time(temp),i2(temp),'Color',color_2014b_orange)
    plot(time(temp),i3(temp),'Color',color_2014b_green)
    xlabel('Time [s]')
    ylabel('Stator Current [A]')
    legend('{\iti}_{s.1}','{\iti}_{s.2}','{\iti}_{s.3}')
subplot(2,2,3)
    hold on, grid on;
    plot(time(temp),n(temp),'Color',color_2014b_blue)
    xlabel('Time [s]')
    ylabel('Rotor Speed [rpm]')
    legend('{\it\Omega}_{r}')
subplot(2,2,4)
    hold on, grid on;
    plot(time(temp),p_noload(temp)/1e3,'Color',color_2014b_blue)
    plot(time(temp),q_noload(temp)/1e3,'Color',color_2014b_orange)
    plot(time(temp),P_noload*ones(1,length(time(temp)))/1e3,'Color',color_2014b_blue,'LineStyle','--')
    plot(time(temp),Q_noload*ones(1,length(time(temp)))/1e3,'Color',color_2014b_orange,'LineStyle','--')
    xlabel('Time [s]')
    ylabel('Power [kW] [kVar]')
    legend('{\itp}_{s}','{\itq}_{s}','{\itP}_{s}','{\itQ}_{s}')

result_noload = {
                    'U_rms_noload' , U_rms_noload , 'V';   
                    'I_rms_noload' , I_rms_noload , 'A'; 
                    'P_noload'     , P_noload/1e3 , 'kW';    
                    'Q_noload'     , Q_noload/1e3 , 'kVar';     
                };

disp(result_noload);

%% EDIT HERE - Parameter Calculation

% -- To Get R1=R_s from Multimeter -- %

R1 = 1.267;          % [Ohm] stator resistance



% -- To Get R1 , L_sl and L_rl from Locked-Rotor Test -- %
% Useful variables from workspace:
% P_lockedrotor: avearge three phase active power to machine
% Q_lockedrotor: avearge three phase reactive power to machine
% I_rms_lockedrotor: phase rms current to machine

R_series = 1;       % [Ohm] stator & rotor resistance in series (R1+R'2)
X_series = 1;       % [Ohm] stator & rotor leakage reactance in series (X1+X2)
L_series = 1;       % [H]   stator & rotor leakage inductance in series

R2  = 1;           % [Ohm] rotor resistance
L1 = 1;            % [H]   stator leakage inductance
L2 = 1;            % [H]   rotor leakage inductance



% -- To Get L_Mag from No-Load Test -- %
% Useful variables from workspace:
% P_noload: avearge three phase active power to machine
% Q_noload: avearge three phase reactive power to machine
% U_rms_noload: phase rms voltage to machine

Rfe = 1;
Xm = 1;                    % [Ohm] magnetizing reactance
Lm = 1;                    % [H]   magnetizing inductance

% -- To Display Parameters -- %

result_parameter = {
                    'R1'   , R1       , 'Ohm'; % [Ohm] stator resistance
                    'R2'   , R2       , 'Ohm'; % [Ohm] rotor resistance
                    'L1'	, L1*1e3  , 'mH';  % [H]   stator leakage inductance
                    'L2'	, L2*1e3  , 'mH';  % [H]   rotor leakage inductance
                    'Rfe'   , Rfe     , 'Ohm'; % [Ohm] core loss resistance
                    'L_m'	, Lm*1e3 , 'mH';  % [H]   magnetizing inductance
                };

disp(result_parameter);

