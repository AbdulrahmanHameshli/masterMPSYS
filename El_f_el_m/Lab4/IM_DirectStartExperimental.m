
% MATLAB Code - IM Parameter Post Process Code

% EEN155 Electric Drives and fields
% Division of Electric Power Engineering
% Chalmers University of Technology 

% Verion 1.0    2018-10-08    Junfei Tang <junfei.tang@chalmers.se>
% Verion 1.1    2018-10-11    Junfei Tang <junfei.tang@chalmers.se>
% Verion 1.2    2018-10-12    Junfei Tang <junfei.tang@chalmers.se>
% Verion 1.3    2022-02-28    Emma Grunditz <emma.grunditz@chalmers.se>


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
% (1) One test is performed: Direct stsart of the induction machine.

%%

close all
clear
clc

figure_configuration_code

% Scaling

K_voltage        =  650 / 5;
K_current        =   35 / 5;
K_speed          = 1500 / 150;


%% Taest Direct Start

data = load('Test_4_IM_Direct_Start');
sample_stop=5000;

data = data';
time = data(1:sample_stop,1);
u1   = data(1:sample_stop,2);
u2   = data(1:sample_stop,3);
u3   = data(1:sample_stop,4);
i1   = data(1:sample_stop,5);
i2   = data(1:sample_stop,6);
i3   = data(1:sample_stop,7);
ua   = data(1:sample_stop,8);
ia   = data(1:sample_stop,9);
If   = data(1:sample_stop,10);
n    = data(1:sample_stop,11);
i6   = data(1:sample_stop,12);

% For t < 0.047 s the machine is at standing still
temp = time < 0.047;
SpeedOffSet = mean(n(temp));
n = n - SpeedOffSet;
% For t > 0.4 s the machine is in steady-state at no-load operation
temp = time > 0.4;
SpeedGain = 1500/mean(n(temp));
n = n * SpeedGain;

% -- Plot -- %

figure('Name','Direct Start - all three phases')
clf;
subplot(2,2,1)
hold on; grid on;
    plot(time,u1,'Color',color_2014b_blue)
    plot(time,u2,'Color',color_2014b_orange)
    plot(time,u3,'Color',color_2014b_green)
    legend('{\itu}_{s.1}','{\itu}_{s.2}','{\itu}_{s.3}')
    xlabel('Time [s]')
    ylabel('Stator Voltage (V)')
subplot(2,2,2)
hold on; grid on;
    plot(time,i1,'Color',color_2014b_blue)
    plot(time,i2,'Color',color_2014b_orange)
    plot(time,i3,'Color',color_2014b_green)
    plot(time,i6,'--','Color',color_2014b_green)
    legend('{\iti}_{s.1}','{\iti}_{s.2}','{\iti}_{s.3}','i_6')
    xlabel('Time [s]')
    title('Stator Current [A]')
subplot(2,2,3)
hold on; grid on;
    plot(time,n,'Color',color_2014b_blue)
    xlabel('Time [s]')
    ylabel('Rotor Speed [rpm]')
    legend('{\it\Omega}_{r}')
subplot(2,2,4)
hold on; grid on;
    plot(time,ia,'Color',color_2014b_red)
    xlabel('Time [s]')
    ylabel('DC Armature Current [A]')
    legend('{\iti}_{a}')


    
figure('Name','Direct Start - one phase')
clf;
subplot(2,2,1)
hold on; grid on;
    plot(time,u3,'Color',color_2014b_green)
    xlabel('Time [s]')
    ylabel('Stator Voltage (V)')
    legend('{\itu}_{s.3}')
subplot(2,2,2)
hold on; grid on;
    plot(time,i6,'Color',color_2014b_green)
    xlabel('Time [s]')
    ylabel('Stator Current [A]')
    legend('{\iti}_{s.3}')
subplot(2,2,3)
hold on; grid on;
    plot(time,n,'Color',color_2014b_blue)
    xlabel('Time [s]')
    ylabel('Rotor Speed [rpm]')
    legend('{\it\Omega}_{r}')
subplot(2,2,4)
hold on; grid on;
    plot(time,ia,'Color',color_2014b_red)
    xlabel('Time [s]')
    ylabel('DC Armature Current [A]')
    legend('{\iti}_{a}')

