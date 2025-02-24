%% SETUP_LAB_HELI_3D
%
% This script sets the model parameters and designs a PID position
% controller using LQR for the Quanser 3-DOF Helicopter plant.
%
% Copyright (C) 2007 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
clear all;
%
%% ############### USER-DEFINED 3-DOF HELI CONFIGURATION ###############
% Amplifier Gain used for yaw and pitch axes: set VoltPAQ to 3.
K_AMP = 3;
% Amplifier Maximum Output Voltage (V)
VMAX_AMP = 24;
% Digital-to-Analog Maximum Voltage (V): set to 10 for Q4/Q8 cards
VMAX_DAC = 10;
% Initial elvation angle (rad)
elev_0 = -27.5*pi/180;
%
%% ############### USER-DEFINED CONTROLLER/FILTER DESIGN ###############
% Type of Controller: set it to 'LQR_AUTO' or 'MANUAL'  
CONTROLLER_TYPE = 'LQG';    % controller design: manual mode
% Is Active Disturbance Mass used: 'YES' or 'NO'
% note: larger gain controller calculated for ADS system
WITH_ADS = 'NO';
% Specifications of a second-order low-pass filter
wcf = 2 * pi * 20; % filter cutting frequency
zetaf = 0.9;        % filter damping ratio
% Anti-windup: integrator saturation (V)
SAT_INT_ERR_ELEV = 7.5;
SAT_INT_ERR_TRAVEL = 7.5;
%
%% ############### USER-DEFINED DESIRED COMMAND SETTINGS ###############
% Note: These limits are imposed on both the program and joystick commands.
% Elevation position command limit (deg)
CMD_ELEV_POS_LIMIT_LOWER = elev_0*180/pi;
CMD_ELEV_POS_LIMIT_UPPER = -CMD_ELEV_POS_LIMIT_LOWER;
% Maximum Rate of Desired Position (rad/s)
CMD_RATE_LIMIT = 45.0 * pi / 180;
%
%% ############### USER-DEFINED JOYSTICK SETTINGS ###############
% Joystick input X sensitivity used for travel (deg/s/V)
K_JOYSTICK_X = 40.0;
% Joystick input Y sensitivity used for elevation (deg/s/V)
K_JOYSTICK_Y = 45.0;
%
%% ############### ACTIVE DISTURBANCE SYSTEM ###############
% Amplifier Gain for ADS: set VoltPAQ to 3.
K_AMP_X = 3;
% Maximum Output Voltage (V)
VMAX_AMP_X = 15;
% Load ADS parameters
[ K_EC_X, POS_HOME, CAL_VEL, CAL_TIME, wcf_x, kp, X_MAX, V_MAX  ] = setup_ads_configuration( );
%
%% ############### MODELING ###############
% These parameters are used for model representation and controller design.
[ Kf, m_h, m_w, m_f, m_b, Lh, La, Lw, g, K_EC_T, K_EC_P, K_EC_E ] = setup_heli_3d_configuration();
%
% For the following state vector: X = [ elevation; pitch; travel, elev_dot, pitch_dot, travel_dot]
% Initialization the state-Space representation of the open-loop System
HELI3D_ABCD_eqns;
% Augment state: Xi = [ elevation; pitch; travel, elev_dot, pitch_dot, travel_dot, elev_int, travel_int]
Ai = A;
Ai(7,1) = 1; % elevation integrator 
Ai(8,3) = 1; % travel integrator 
Ai(8,8) = 0;
Bi = B;
Bi(8,2) = 0;
%
%% ############### DISPLAY ###############
if strcmp ( CONTROLLER_TYPE, 'LQR' )
    % LQR Controller Design Specifications
    if strcmp(WITH_ADS, 'NO')
        Q = diag([100 1 10 0 0 2 10 0.1]);
        R = 0.05*diag([1 1]);        
    elseif strcmp(WITH_ADS, 'YES')
        Q = diag([100 1 10 0 0 2 10 0.1]);
        R = 0.025*diag([1 1]);    
    end
    % Automatically calculate the LQR controller gain
    K = lqr( Ai, Bi, Q, R ); 
    
    % Kalman Filter
    W = diag([1e3 1e3 1e3 1e-3 1e-3 1e-3]);
    V = eye(3);
    N = zeros(6,3);
    sys = ss(A,[B,eye(6)],C,[D zeros(3,6)]);
    Kest = kalman(sys,W,V,N);
    A_kal = Kest.a;
    B_kal = Kest.b;
    C_kal = eye(6);
    D_kal = zeros(6,5);
    
    % Display the calculated gains
    disp( ' ' )
    disp( 'Calculated LQR controller gain elements: ' )
    K    
    
 elseif strcmp ( CONTROLLER_TYPE, 'LQ' )
    % Automatically calculate the LQ controller gain
    Q = diag([100 1 10 0 0 2]);
    R = 0.05*diag([1 1]);    
    L = lqr( A, B, Q, R );
    [U_svd S_svd V_svd]= svd(C*inv(A-B*L)*B,0);
    Kr = -V_svd*inv(S_svd)*U_svd';
    
    
    % Kalman Filter
    W = diag([1e3 1e3 1e3 1e-3 1e-3 1e-3]);
    V = eye(3);
    N = zeros(6,3);
    sys = ss(A,[B,eye(6)],C,[D zeros(3,6)]);
    Kest = kalman(sys,W,V,N);
    A_kal = Kest.a;
    B_kal = Kest.b;
    C_kal = eye(6);
    D_kal = zeros(6,5);
    
    disp( ' ' )
    disp( 'Calculated LQ controller gain elements: ' )
    L
    Kr
    
elseif strcmp ( CONTROLLER_TYPE, 'LQI' )
    % Automatically calculate the LQI controller gain
    Ae = [A zeros(6,3);...
         -C zeros(3,3)];
    Be = [B;zeros(3,2)];
    Q = diag([100 1 10 0 0 2 10 0.1 0.1]);
    R = 0.05*diag([1 1]);
    Le = lqr( Ae, Be, Q, R );
    L = Le(:,1:6);
    Li = Le(:,7:9);
    disp( ' ' )
    disp( 'Calculated LQI controller gain elements: ' )
    L
    
    elseif strcmp ( CONTROLLER_TYPE, 'LQIF' )
    % Automatically calculate the LQIF controller gain
    Ae = [A zeros(6,3);...
         -C zeros(3,3)];
    Be = [B;zeros(3,2)];
    Q = diag([100 1 10 0 0 2 10 0.1 0.1]);
    R = 0.05*diag([1 1]);
    Le = lqr( Ae, Be, Q, R );
    L = Le(:,1:6);
    Li = Le(:,7:9);
    A_tot = [A B; C D];
    [U_svd S_svd V_svd]= svd(A_tot,0);
    A_tot_inv = V_svd*inv(S_svd)*U_svd';
    Kr =[L eye(2,2)]*A_tot_inv*[zeros(6,3);eye(3,3)];
    
     % Kalman Filter
    W = diag([1e3 1e3 1e3 1e-3 1e-3 1e-3]);
    V = eye(3);
    N = zeros(6,3);
    sys = ss(A,[B,eye(6)],C,[D zeros(3,6)]);
    Kest = kalman(sys,W,V,N);
    A_kal = Kest.a;
    B_kal = Kest.b;
    C_kal = eye(6);
    D_kal = zeros(6,5);
    
    disp( ' ' )
    disp( 'Calculated LQIF controller gain elements: ' )
    Le
    Kr
    
     elseif strcmp ( CONTROLLER_TYPE, 'LQG' )
    % Automatically calculate the LQG controller gain
    Q = diag([100 1 10 0 0 2]);
    R = 0.05*diag([1 1]);    
    L = lqr( A, B, Q, R );
    [U_svd S_svd V_svd]= svd(C*inv(A-B*L)*B,0);
    Kr = -V_svd*inv(S_svd)*U_svd';
    
     % Kalman Filter
    W = diag([1e3 1e3 1e3 1e-3 1e-3 1e-3]);
    V = eye(3);
    N = zeros(6,3);
    sys = ss(A,[B,eye(6)],C,[D zeros(3,6)]);
    Kest = kalman(sys,W,V,N);
    A_kal = Kest.a;
    B_kal = Kest.b;
    C_kal = eye(6);
    D_kal = zeros(6,5);
    
    disp( ' ' )
    disp( 'Calculated LQG controller gain elements: ' )
    L
    Kr
   
    
    elseif strcmp ( CONTROLLER_TYPE, 'PD' )
    
    % Automatically calculate the PI controller gain
    Q = diag([100 1 10 0 0 2 10 0.1]);
    R = 0.05*diag([1 1]);    
    K = lqr( Ai, Bi, Q, R );
    K(1,7)=0;
    K(1,8)=0;
    K(2,7)=0;
    K(2,8)=0;
    
    disp( ' ' )
    disp( 'Calculated P controller gain elements: ' )
    K  
    
    elseif strcmp ( CONTROLLER_TYPE, 'PI' )
    
    % Automatically calculate the PI controller gain
    Q = diag([100 1 10 0 0 2 10 0.1]);
    R = 0.05*diag([1 1]);    
    K = lqr( Ai, Bi, Q, R );
    K(1,4)=0;
    K(1,5)=0;
    K(1,6)=0;
    K(2,4)=0;
    K(2,5)=0;
    K(2,6)=0;
  
    disp( ' ' )
    disp( 'Calculated PI controller gain elements: ' )
    K  
    
 elseif strcmp ( CONTROLLER_TYPE, 'P' )
    % Automatically calculate the P controller gain
    Q = diag([100 1 10 0 0 2 10 0.1]);
    R = 0.05*diag([1 1]);    
    K = lqr( Ai, Bi, Q, R );
    K(1,4)=0;
    K(1,5)=0;
    K(1,6)=0;
    K(1,7)=0;
    K(1,8)=0;
    K(2,4)=0;
    K(2,5)=0;
    K(2,6)=0;
    K(2,7)=0;
    K(2,8)=0;
    disp( ' ' )
    disp( 'Calculated P controller gain elements: ' )
    K  
    
else
    error( 'Error: Please set the type of controller that you wish to implement.' )
end
