    %% 3-DOF helicopter nonlinear model parameters %%

Mf = 0.71; %[kg] Mass of front propeller assembly
Mb = 0.71; %[kg] Mass of back propeller assembly
Mc = 1.69; %[kg] Mass of the counterwight
Ld = 0.05; %[m] Length of pendulum for elevention axis
Lc = 0.44; %[m] Distance from pivot point to counterweight
La = 0.62; %[m] Distance from pivot point to helicopter body
Le = 0.02; %[m] Length of pendulum for pitch axis
Lh = 0.18; %[m] Distance from pitch axis to motor
g = 9.81; %[m/s^2] Gravitational acceleration
Km = 0.12; %[N/V] Propeller force-thrust constant
Jeps = 0.86; %[kg*m^2] Moment of inertia level axis
Jro = 0.044; %[kg*m^2] Moment of inertia pitch axis
Jlam = 0.82; %[kg*m^2] Moment of inertia travel axis
Neps = 0.001; %[kg*m^2/s] Viscous friction level axis
Nro = 0.001; %[kg*m^2/s] Viscous friction pitch axis
Nlam = 0.005; %[kg*m^2/s] Viscous friction travel axis

% Coefficients p
p1 = (-(Mf+Mb)*g*La+Mc*g*Lc)/Jeps;
p2 = (-(Mf+Mb)*g*(Ld+Le)+Mc*g*Ld)/Jeps;
p3 = -Neps/Jeps;
p4 = Km*La/Jeps;
p5 = (-Mf+Mb)*g*Lh/Jro;
p6 = -(Mf+Mb)*g*Le/Jro;
p7 = -Nro/Jro;
p8 = Km*Lh/Jro;
p9 = -Nlam/Jlam;
p10 = -Km*La/Jlam;

% initial conditions
eps0 = 0;
ro0 = 0;
lam0 = 0;
eps0dot = 0;
ro0dot = 0;
lam0dot = 0;






% 2.a)
syms epsi_dot ro_dot lambda_dot epsi ro lambda Vf Vb t real 


x = [epsi ro lambda epsi_dot ro_dot lambda_dot]';
x0 = [eps0 ro0 lam0 eps0dot ro0dot  lam0dot]';

u = [Vf;Vb];
f = [epsi_dot ;ro_dot ;lambda_dot ;p1*cos(epsi)+p2*sin(epsi)+p3*epsi_dot;p5*cos(ro)+p6*sin(ro)+p7*ro_dot;p9*lambda];
g = [0 0; 0 0; 0 0; p4*cos(ro) p4*cos(ro); p8 -p8; p10*sin(ro) p10*sin(ro)];


xdot = f+g*u;

xdot_func = matlabFunction(xdot,'Vars',{t,x,u});
f__func = matlabFunction(f,'Vars',{t,x});   
g_func= matlabFunction(g,'Vars',{t,x});

u0 = -f__func(0,x0)\g_func(0,x0);


model = 'nlmodel_3D';
io = 'nlmodel_3D/3-DOF NONLINEAR MODEL';
op = operpoint(model); % we could use this instead of the hole math in
% matlab
Vf0= u0(1);
Vb0= u0(2);

linsys_code = linearize(model,io,u0);





% 2.b)

% 2.b) A and B matresis after linear izing on opt point
A = linsys_code.A;
B = linsys_code.B ; 
C = linsys_code.C;
D = linsys_code.D;



% 
% % exercise 4   
open_system("linearized_model");
% simTime = 10; 
% simOut = sim("linearized_model", 'StopTime', num2str(simTime));
% t = simOut.tout;  
% output = simOut.sim_x_nonlin;  
% figure;
% for i = 1:6
%     subplot(6, 2, i);
%     plot(t, output.Data(:, i),"--");
%     xlabel('Time');
%     ylabel(['Output ', num2str(i)]);
%     title(['nonlin - Output ', num2str(i)]);
%     grid on;
% end
% 
% open_system("linearized_model");
% simTime = 2; 
% simOut = sim("linearized_model", 'StopTime', num2str(simTime));
% t = simOut.tout; 
% output = simOut.sim_x_lin;  
% figure;
% for i = 1:6
%     subplot(6, 2, i);
%     plot(t, output.Data(:, i),"--");
%     xlabel('Time');
%     ylabel(['Output ', num2str(i)]);
%     title(['lin - Output ', num2str(i)]);
%     grid on;
% end
%%ans: Very good, linearizing is good.


%Exercise 10


% Controll epsi and lamba
% C_for_sim = [1 0 0 0 0 0; 
%      0 0 1 0 0 0;
%      0 0 0 0 0 0;
%      0 0 0 0 0 0;
%      0 0 0 0 0 0;
%      0 0 0 0 0 0];

C_forKr = [1 0 0 0 0 0; 
     0 0 1 0 0 0];

D = [0 0; 0 0;0 0; 0 0; 0 0; 0 0];

qs = [1000 1000 1 1 1 1];
Qx = diag(qs);
Qu = eye(length(u));


reference_value = [80,360*pi/180];
s = ctrb(A,B);
rref(transpose(s))
[K,S,P] = lqr(A,B,Qx,   Qu);
Kr = -inv(C_forKr*inv(A-B*K)*B);
% 
% 
% 



simTime = 10; 
simOut = sim("nlmodel_3D.mdl", 'StopTime', num2str(simTime));
t = simOut.tout;  
open_system("nlmodel_3D/Nonlin")
open_system("nlmodel_3D/Lin")



