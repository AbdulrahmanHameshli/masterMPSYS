
% MATLAB Code - IM Parameter Post Process Code

% EEN155 Electric Drives and fields
% Division of Electric Power Engineering
% Chalmers University of Technology 

% Verion 1.0    2022-02-28    Emma Grunditz <emma.grunditz@chalmers.se>
% Verion 1.1    2023-02-02    Emma Grunditz <emma.grunditz@chalmers.se>



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
% (1) Exchange the parameter values = 1, with corrent expressions, to plot the
% torque-speed-graph of the induction machine

% IMPORTANT! when using vector arrays in MATLAB, 
% element-vise multiplication must be written  .* 
% and division must be written  ./ 
% and potence must be written  .^
% See example below for the slip-vector, Z2 and Te

% IMPORTANT! The imaginary number i, written using the letter j

%%
close all;
clear;
clc;

%% EDIT HERE - Parameter definitions
U1 = 1;                     % (V) Stator phase voltage rms value

f_s=1;                      % (Hz) synchronous stator voltage frequncy (from grid)
w_s=1;                      % (rad/s) synchronous angular frequncy
ns=1500;                    % (rpm) synchronous rotating speed (=120*f_s/PolePair
n_vect=[0:ns];              % (rpm) speed-vector, 0-syncronous speed
s=(ns-n_vect)./ns;          % (-) slip-vector, 0-syncronous speed  

R1=1;                       % (ohm) Stator winding resistance
R2=1;                       % (ohm) Rotor resistance, referred to stator
L1=1e-3;                    % (H) Stator leakage inductance
L2=1e-3;                    % (H) Rotor leakage inductance, referred to stator
Lm=1e-3;                    % (H) Magnetizing inductance
Rfe= 1;                     % (ohm) Core loss resistance

%% EDIT HERE - Impedances - using jw-method

Z2 = (j.*w_s.*L2) + R2 + R2.*((1-s)./s);        % Total rotor winding impedance
Zm = 1;                     % Total Magnetisering impedance
Z_2m = 1;                   % Z_2m = Z2 // Zm
Z1 = 1;                     % Total stator winding impedance

%% EDIT HERE - Voltages, currents

U_2m = 1;                           % Voltage acoss Z_2m = Z2 // Zm (Tips: spänningsdelning)
I2 = 1;                             % Rotor current

Pe = 1;                           % Mechanical power
Te = Pe./(n_vect.*pi./30);      % Mechanical torque


figure(1)
clf;

    plot(n_vect,Te);grid on; hold on;
    xlabel('Rotor speed [rpm]');
    ylabel('Torque [Nm]');
    title('Stationary model');

%% EDIT HERE - Your Measured Data
% --- ENTER MEASURED DATA HERE ---
n1=1;
n2=1;
n3=1;
n4=1;
n5=1;
Trq1=1;
Trq2=1;
Trq3=1;
Trq4=1;
Trq5=1;

n_meas=[n1,n2,n3,n4,n5];
Trq_meas=[Trq1,Trq2,Trq3,Trq4,Trq5];

plot(n_meas, Trq_meas,'r*-');
