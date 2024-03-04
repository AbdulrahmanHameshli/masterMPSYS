function Qpp=Dynamic_abbIRB4v2(u)
% Dynamic_abbIRB4 Calculates the direct dynamics of robot ABB IRB 120 (4DOF)
%
% This model uses external parameters


%Joint Position
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);

%Joint Velocity
qp1=u(5);
qp2=u(6);
qp3=u(7);
qp4=u(8);

%Gravity Vector
gz=u(9);

%Viscous Friction Matrix
Beta(1,1)=u(10);
Beta(2,2)=u(11);
Beta(3,3)=u(12);
Beta(4,4)=u(13);

%Time
t=u(14);


%Joint Position Vector
Q=[q1; q2; q3; q4];

%Joint Velocity Vector
Qp=[qp1; qp2; qp3; qp4];

% Load Robot Parameters
abb_irb120_params

% Inertia Tensors
I133=I1(6);

I211=I2(1);
I212=I2(2);
I213=I2(3);
I222=I2(4);
I223=I2(5);
I233=I2(6);

I311=I3(1);
I312=I3(2);
I313=I3(3);
I322=I3(4);
I323=I3(5);
I333=I3(6);

I411=I4(1);
I412=I4(2);
I413=I4(3);
I422=I4(4);
I423=I4(5);
I433=I4(6);

%Common substitutions
c2=cos(q2);
s2=sin(q2);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca3=cos(al + q3);
sa3=sin(al + q3);

c2_234=cos(2*q2 + 2*q3 + 2*q4);
s2_234=sin(2*q2 + 2*q3 + 2*q4);

c34=cos(q3 + q4);
s34=sin(q3 + q4);

c2_a23=cos(2*al + 2*q2 + 2*q3);
s2_a23=sin(2*al + 2*q2 + 2*q3);

cam4=cos(al - q4);
sam4=sin(al - q4);

c22=cos(2*q2);
s22=sin(2*q2);

ca_223_4=cos(al + 2*q2 + 2*q3 + q4);

c22_34=cos(2*q2 + q3 + q4);


% [M, C, G] = Get_dynamics(u)

%% Inertia Matrix
%TODO: define the inertia Matrix.
% M(1,1)=M(1,1);
% M(1,2)=M(1,2);
% M(1,3)=M(1,3);
% M(1,4)=M(1,4);
% M(2,1)=M(2,1);
% M(2,2)=M(2,2);
% M(2,3)=M(2,3);
% M(2,4)=M(2,4);
% M(3,1)=M(3,1);
% M(3,2)=M(3,2);
% M(3,3)=M(3,3);
% M(3,4)=M(3,4);
% M(4,1)=M(4,1);
% M(4,2)=M(4,2);
% M(4,3)=M(4,3);
% M(4,4)=M(4,4);


M(1,1)= I133 + cos(al + q2 + q3)*(I311*cos(al + q2 + q3) - I312*sin(al + q2 + q3)) - sin(al + q2 + q3)*(I312*cos(al + q2 + q3) - I322*sin(al + q2 + q3)) + cos(q2 + q3 + q4)*(I422*cos(q2 + q3 + q4) + I412*sin(q2 + q3 + q4)) + sin(q2 + q3 + q4)*(I412*cos(q2 + q3 + q4) + I411*sin(q2 + q3 + q4)) + L21^2*m2 + I211*cos(q2)^2 + I222*sin(q2)^2 + m3*cos(q1)^2*(L2*sin(q2) + L51*sin(al + q2 + q3))^2 + m3*sin(q1)^2*(L2*sin(q2) + L51*sin(al + q2 + q3))^2 - 2*I212*cos(q2)*sin(q2) - L21^2*m2*cos(q2)^2 + m4*cos(q1)^2*(L2*sin(q2) + L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3))^2 + m4*sin(q1)^2*(L2*sin(q2) + L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3))^2;
M(1,2)= I213*cos(q2) - I223*sin(q2) + I313*cos(al + q2 + q3) - I423*cos(q2 + q3 + q4) - I323*sin(al + q2 + q3) - I413*sin(q2 + q3 + q4);
M(1,3)= I313*cos(al + q2 + q3) - I423*cos(q2 + q3 + q4) - I323*sin(al + q2 + q3) - I413*sin(q2 + q3 + q4);
M(1,4)= -I423*cos(q2 + q3 + q4) - I413*sin(q2 + q3 + q4);
M(2,1)= I213*cos(q2) - I223*sin(q2) + I313*cos(al + q2 + q3) - I423*cos(q2 + q3 + q4) - I323*sin(al + q2 + q3) - I413*sin(q2 + q3 + q4);
M(2,2)= I233 + I333 + I433 + L2^2*m3 + L2^2*m4 + L7^2*m4 + L21^2*m2 + L41^2*m4 + L51^2*m3 - 2*L2*L41*m4*sin(q3 + q4) + 2*L7*L41*m4*sin(al - q4) + 2*L2*L7*m4*cos(al + q3) + 2*L2*L51*m3*cos(al + q3);
M(2,3)= m4*L7^2 + 2*m4*sin(al - q4)*L7*L41 + L2*m4*cos(al + q3)*L7 + m4*L41^2 - L2*m4*sin(q3 + q4)*L41 + m3*L51^2 + L2*m3*cos(al + q3)*L51 + I333 + I433;
M(2,4)= I433 + L41^2*m4 - L2*L41*m4*sin(q3 + q4) + L7*L41*m4*sin(al - q4);
M(3,1)= I313*cos(al + q2 + q3) - I423*cos(q2 + q3 + q4) - I323*sin(al + q2 + q3) - I413*sin(q2 + q3 + q4);
M(3,2)= m4*L7^2 + 2*m4*sin(al - q4)*L7*L41 + L2*m4*cos(al + q3)*L7 + m4*L41^2 - L2*m4*sin(q3 + q4)*L41 + m3*L51^2 + L2*m3*cos(al + q3)*L51 + I333 + I433;
M(3,3)= m4*L7^2 + 2*m4*sin(al - q4)*L7*L41 + m4*L41^2 + m3*L51^2 + I333 + I433;
M(3,4)= m4*L41^2 + L7*m4*sin(al - q4)*L41 + I433;
M(4,1)= -I423*cos(q2 + q3 + q4) - I413*sin(q2 + q3 + q4);
M(4,2)= I433 + L41^2*m4 - L2*L41*m4*sin(q3 + q4) + L7*L41*m4*sin(al - q4);
M(4,3)= m4*L41^2 + L7*m4*sin(al - q4)*L41 + I433;
M(4,4)= m4*L41^2 + I433;
%% Centripetal and Coriolis Matrix
%TODO: define the C matrix
% C(1,1)=C(1,1);
% C(1,2)=C(1,2);
% C(1,3)=C(1,3);
% C(1,4)=C(1,4);
% C(2,1)=C(2,1);
% C(2,2)=C(2,2);
% C(2,3)=C(2,3);
% C(2,4)=C(2,4);
% C(3,1)=C(3,1);
% C(3,2)=C(3,2);
% C(3,3)=C(3,3);
% C(3,4)=C(3,4);
% C(4,1)=C(4,1);
% C(4,4)=C(4,4);
% C(4,2)=C(4,2);
% C(4,3)=C(4,3);

C(1,1)=I412*qp2*cos(2*q2 + 2*q3 + 2*q4) - I312*qp3*cos(2*al + 2*q2 + 2*q3) - I212*qp2*cos(2*q2) - I312*qp2*cos(2*al + 2*q2 + 2*q3) + I412*qp3*cos(2*q2 + 2*q3 + 2*q4) + I412*qp4*cos(2*q2 + 2*q3 + 2*q4) - (I311*qp2*sin(2*al + 2*q2 + 2*q3))/2 - (I311*qp3*sin(2*al + 2*q2 + 2*q3))/2 + (I322*qp2*sin(2*al + 2*q2 + 2*q3))/2 + (I322*qp3*sin(2*al + 2*q2 + 2*q3))/2 - (I211*qp2*sin(2*q2))/2 + (I222*qp2*sin(2*q2))/2 + (I411*qp2*sin(2*q2 + 2*q3 + 2*q4))/2 + (I411*qp3*sin(2*q2 + 2*q3 + 2*q4))/2 + (I411*qp4*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp2*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp3*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp4*sin(2*q2 + 2*q3 + 2*q4))/2 + (L7^2*m4*qp2*sin(2*al + 2*q2 + 2*q3))/2 + (L7^2*m4*qp3*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*qp2*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*qp3*sin(2*al + 2*q2 + 2*q3))/2 + (L2^2*m3*qp2*sin(2*q2))/2 + (L2^2*m4*qp2*sin(2*q2))/2 + (L21^2*m2*qp2*sin(2*q2))/2 - (L41^2*m4*qp2*sin(2*q2 + 2*q3 + 2*q4))/2 - (L41^2*m4*qp3*sin(2*q2 + 2*q3 + 2*q4))/2 - (L41^2*m4*qp4*sin(2*q2 + 2*q3 + 2*q4))/2 + L7*L41*m4*qp2*cos(al + 2*q2 + 2*q3 + q4) + L7*L41*m4*qp3*cos(al + 2*q2 + 2*q3 + q4) + (L7*L41*m4*qp4*cos(al + 2*q2 + 2*q3 + q4))/2 + L2*L41*m4*qp2*cos(2*q2 + q3 + q4) + (L2*L41*m4*qp3*cos(2*q2 + q3 + q4))/2 + (L2*L41*m4*qp4*cos(2*q2 + q3 + q4))/2 + L2*L7*m4*qp2*sin(al + 2*q2 + q3) + (L2*L7*m4*qp3*sin(al + 2*q2 + q3))/2 + L2*L51*m3*qp2*sin(al + 2*q2 + q3) + (L2*L51*m3*qp3*sin(al + 2*q2 + q3))/2 - (L2*L41*m4*qp3*cos(q3 + q4))/2 - (L2*L41*m4*qp4*cos(q3 + q4))/2 - (L2*L7*m4*qp3*sin(al + q3))/2 - (L2*L51*m3*qp3*sin(al + q3))/2 - (L7*L41*m4*qp4*cos(al - q4))/2;
C(1,2)=I423*qp2*sin(q2 + q3 + q4) - I323*qp3*cos(al + q2 + q3) - I413*qp2*cos(q2 + q3 + q4) - I413*qp3*cos(q2 + q3 + q4) - I413*qp4*cos(q2 + q3 + q4) - I313*qp2*sin(al + q2 + q3) - I313*qp3*sin(al + q2 + q3) - I312*qp1*cos(2*al + 2*q2 + 2*q3) - I323*qp2*cos(al + q2 + q3) + I423*qp3*sin(q2 + q3 + q4) + I423*qp4*sin(q2 + q3 + q4) - I212*qp1*cos(2*q2) + I412*qp1*cos(2*q2 + 2*q3 + 2*q4) - (I311*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (I322*qp1*sin(2*al + 2*q2 + 2*q3))/2 - (I211*qp1*sin(2*q2))/2 + (I222*qp1*sin(2*q2))/2 + (I411*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - I223*qp2*cos(q2) - I213*qp2*sin(q2) + (L7^2*m4*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (L2^2*m3*qp1*sin(2*q2))/2 + (L2^2*m4*qp1*sin(2*q2))/2 + (L21^2*m2*qp1*sin(2*q2))/2 - (L41^2*m4*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 + L7*L41*m4*qp1*cos(al + 2*q2 + 2*q3 + q4) + L2*L41*m4*qp1*cos(2*q2 + q3 + q4) + L2*L7*m4*qp1*sin(al + 2*q2 + q3) + L2*L51*m3*qp1*sin(al + 2*q2 + q3);
C(1,3)=I423*qp2*sin(q2 + q3 + q4) - I323*qp3*cos(al + q2 + q3) - I413*qp2*cos(q2 + q3 + q4) - I413*qp3*cos(q2 + q3 + q4) - I413*qp4*cos(q2 + q3 + q4) - I313*qp2*sin(al + q2 + q3) - I313*qp3*sin(al + q2 + q3) - I312*qp1*cos(2*al + 2*q2 + 2*q3) - I323*qp2*cos(al + q2 + q3) + I423*qp3*sin(q2 + q3 + q4) + I423*qp4*sin(q2 + q3 + q4) + I412*qp1*cos(2*q2 + 2*q3 + 2*q4) - (I311*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (I322*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (I411*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 + (L7^2*m4*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*qp1*sin(2*al + 2*q2 + 2*q3))/2 - (L41^2*m4*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 + L7*L41*m4*qp1*cos(al + 2*q2 + 2*q3 + q4) + (L2*L41*m4*qp1*cos(2*q2 + q3 + q4))/2 + (L2*L7*m4*qp1*sin(al + 2*q2 + q3))/2 + (L2*L51*m3*qp1*sin(al + 2*q2 + q3))/2 - (L2*L41*m4*qp1*cos(q3 + q4))/2 - (L2*L7*m4*qp1*sin(al + q3))/2 - (L2*L51*m3*qp1*sin(al + q3))/2;
C(1,4)=I423*qp2*sin(q2 + q3 + q4) - I413*qp3*cos(q2 + q3 + q4) - I413*qp4*cos(q2 + q3 + q4) - I413*qp2*cos(q2 + q3 + q4) + I423*qp3*sin(q2 + q3 + q4) + I423*qp4*sin(q2 + q3 + q4) + I412*qp1*cos(2*q2 + 2*q3 + 2*q4) + (I411*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - (L41^2*m4*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 + (L7*L41*m4*qp1*cos(al + 2*q2 + 2*q3 + q4))/2 + (L2*L41*m4*qp1*cos(2*q2 + q3 + q4))/2 - (L2*L41*m4*qp1*cos(q3 + q4))/2 - (L7*L41*m4*qp1*cos(al - q4))/2;
C(2,1)=-(qp1*(2*I412*cos(2*q2 + 2*q3 + 2*q4) - 2*I212*cos(2*q2) - 2*I312*cos(2*al + 2*q2 + 2*q3) - I311*sin(2*al + 2*q2 + 2*q3) + I322*sin(2*al + 2*q2 + 2*q3) - I211*sin(2*q2) + I222*sin(2*q2) + I411*sin(2*q2 + 2*q3 + 2*q4) - I422*sin(2*q2 + 2*q3 + 2*q4) + L7^2*m4*sin(2*al + 2*q2 + 2*q3) + L51^2*m3*sin(2*al + 2*q2 + 2*q3) + L2^2*m3*sin(2*q2) + L2^2*m4*sin(2*q2) + L21^2*m2*sin(2*q2) - L41^2*m4*sin(2*q2 + 2*q3 + 2*q4) + 2*L7*L41*m4*cos(al + 2*q2 + 2*q3 + q4) + 2*L2*L41*m4*cos(2*q2 + q3 + q4) + 2*L2*L7*m4*sin(al + 2*q2 + q3) + 2*L2*L51*m3*sin(al + 2*q2 + q3)))/2;
C(2,2)=- L2*qp3*(L41*m4*cos(q3 + q4) + L7*m4*sin(al + q3) + L51*m3*sin(al + q3)) - L41*m4*qp4*(L2*cos(q3 + q4) + L7*cos(al - q4));
C(2,3)=- L2*qp2*(L41*m4*cos(q3 + q4) + L7*m4*sin(al + q3) + L51*m3*sin(al + q3)) - L2*qp3*(L41*m4*cos(q3 + q4) + L7*m4*sin(al + q3) + L51*m3*sin(al + q3)) - L41*m4*qp4*(L2*cos(q3 + q4) + L7*cos(al - q4));
C(2,4)=-L41*m4*(L2*cos(q3 + q4) + L7*cos(al - q4))*(qp2 + qp3 + qp4);
C(3,1)=-(qp1*(2*I412*cos(2*q2 + 2*q3 + 2*q4) - 2*I312*cos(2*al + 2*q2 + 2*q3) - I311*sin(2*al + 2*q2 + 2*q3) + I322*sin(2*al + 2*q2 + 2*q3) + I411*sin(2*q2 + 2*q3 + 2*q4) - I422*sin(2*q2 + 2*q3 + 2*q4) + L7^2*m4*sin(2*al + 2*q2 + 2*q3) + L51^2*m3*sin(2*al + 2*q2 + 2*q3) - L41^2*m4*sin(2*q2 + 2*q3 + 2*q4) - L2*L41*m4*cos(q3 + q4) - L2*L7*m4*sin(al + q3) - L2*L51*m3*sin(al + q3) + 2*L7*L41*m4*cos(al + 2*q2 + 2*q3 + q4) + L2*L41*m4*cos(2*q2 + q3 + q4) + L2*L7*m4*sin(al + 2*q2 + q3) + L2*L51*m3*sin(al + 2*q2 + q3)))/2;
C(3,2)=L2*qp2*(L41*m4*cos(q3 + q4) + L7*m4*sin(al + q3) + L51*m3*sin(al + q3)) - L7*L41*m4*qp4*cos(al - q4);
C(3,3)=-L7*L41*m4*qp4*cos(al - q4);
C(3,4)=-L7*L41*m4*cos(al - q4)*(qp2 + qp3 + qp4);
C(4,1)=-(qp1*(2*I412*cos(2*q2 + 2*q3 + 2*q4) + I411*sin(2*q2 + 2*q3 + 2*q4) - I422*sin(2*q2 + 2*q3 + 2*q4) - L41^2*m4*sin(2*q2 + 2*q3 + 2*q4) - L2*L41*m4*cos(q3 + q4) - L7*L41*m4*cos(al - q4) + L7*L41*m4*cos(al + 2*q2 + 2*q3 + q4) + L2*L41*m4*cos(2*q2 + q3 + q4)))/2;
C(4,2)=L41*m4*qp2*(L2*cos(q3 + q4) + L7*cos(al - q4)) + L7*L41*m4*qp3*cos(al - q4);
C(4,3)=L7*L41*m4*cos(al - q4)*(qp2 + qp3);
C(4,4)=0;

%% Gravitational Torques Vector
%TODO: define the G vector
% G(1,1)=G(1,1);
% G(2,1)=G(2,1);
% G(3,1)=G(3,1);
% G(4,1)=G(4,1);
G(1,1)=0;
G(2,1)=- gz*m4*(L2*sin(q2) + L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3)) - gz*m3*(L2*sin(q2) + L51*sin(al + q2 + q3)) - L21*gz*m2*sin(q2);
G(3,1)=- gz*m4*(L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3)) - L51*gz*m3*sin(al + q2 + q3);
G(4,1)=-L41*gz*m4*cos(q2 + q3 + q4);

%TODO: define the external torque (4x1) to simulate the natural dynamics of the
%robot
Tau=zeros(4,1);

%TODO: compute the direct dynamics using M, C, G, B, and Tau
Fv=diag([u(10),u(11),u(12),u(13)]);
Qpp=inv(M) * (Tau - C * Qp - G - Fv * Qp);


end 

% function [M, C, G] = Get_dynamics(u)
% 
% joints={'R','R','R','R'};
% n = 4
% %Joint Position
% q1=u(1);
% q2=u(2);
% q3=u(3);
% q4=u(4);
% 
% %Joint Velocity
% qp1=u(5);
% qp2=u(6);
% qp3=u(7);
% qp4=u(8);
% 
% %Gravity Vector
% gz=u(9);
% 
% %Viscous Friction Matrix
% Beta(1,1)=u(10);
% Beta(2,2)=u(11);
% Beta(3,3)=u(12);
% Beta(4,4)=u(13);
% 
% %Time
% t=u(14);
% 
% 
% %Joint Position Vector
% Q=[q1; q2; q3; q4];
% 
% %Joint Velocity Vector
% Qp=[qp1; qp2; qp3; qp4];
% n = 4
% L1 = 0.29;
%     L2 = 0.27;
%     L3 = 0.07;
%     L4 = 0.302;
%     L5 = 0.072;
%     L6 = 0.1;
% L7=0.31;
% L8=0.172;
% al=1.343;
% 
%     L11 = L1/2;
%     L21 = L2/2;
%     L31 = L3/2;
%     L32 = L4/2;
%     L41 = L8/2;
% L51=0.155;
% 
% H1_0 = dhHT([q1,L1, 0, -pi/2]);
% H2_1 = dhHT([q2-pi/2, 0,L2, 0]);
% H3_2 = dhHT([q3+al, 0, L7, 0]);
% Hv2_v1 = dhHT([q4-al+pi/2,0,L8+L6,0]);
% Hef_v2 = dhHT([pi/2,0,0,pi/2]);
% % Homogeneous Transformations links
% H{1} = H1_0;
% H{2} = H2_1;
% H{3} = H3_2;
% H{4} = Hv2_v1;
% 
% % Homogeneous Transformations cm
% 
% Hcm1_0 = dhHT([q1, L11, 0, 0]);
% Hcm2_1  = dhHT([q2-pi/2, 0, L21, 0]);
% Hcm3_2  = dhHT([q3+al, 0, L51, 0]);
% Hcm4_3  = dhHT([q4-al+pi/2, 0, L41, 0]);
% 
% Hc{1} = Hcm1_0;
% Hc{2} = Hcm2_1;
% Hc{3} = Hcm3_2;
% Hc{4} = Hcm4_3;
% 
% aH{1}=H{1};
% 
% 
% for i=2:n
%     aH{i}=aH{i-1}*H{i};
% end
% 
% aHc{1}=Hc{1};
% for i=2:n
%     aHc{i}=(aH{i-1}*Hc{i});
% end
% 
% 
% aHd = aHc
% 
% 
% 
% syms z3d real
% 
% z{1}=[0;0;1];
% t{1}=[0;0;0];
% 
% for i=1:n
%     
%     % axis of motion third 3D column vector absolute H
%     z{i+1}=aH{i}(1:3,3);
%     % link position vector fourth 3D column vector absolute H
%     t{i+1}=aH{i}(1:3,4);
% 
%     % cm position vector fourth 3D column vector absolute H
%     tc{i+1}=aHc{i}(1:3,4);
% 
%     % Rotiation Matrix CM
%     Rc{i}=aHc{i}(1:3,1:3);
% end
% 
% syms Jef_0 real
% 
% Jef_0 = Jac(t,z,t{end},n,joints);
% 
% 
% % Jval=([diff(t{end},q1), diff(t{end},q2), diff(t{end},q3), diff(t{end},q4)]);
% 
% % DJ=(Jef_0(1:3,:)-Jval);
% 
% syms Jcm_0 J real
% 
% Jcm_0={'','','',''};
% 
% for j=1:n
%     Jcm_0{j}=Jac(t,z,tc{j+1},j,joints);
%     % fprintf('J_cm for link %i ', j);
%     % disp(Jcm_0{j})
% 
%     % Validation
%     % Jval=simplify([diff(tc{j+1},q1), diff(tc{j+1},q2)]);
%     % DJcm=simplify(Jcm_0{j}(1:3,:)-Jval)
% end
% % clear all
% close all
% clc
% 
% 
% syms q1 q2 q3 q4 qp1 qp2 qp3 qp4 gz real
% syms Beta_11 Beta_22 Beta_33 Beta_44 real
% syms t real
% syms m1 m2 m3 m4 real
% 
% T0_W = eye(4);
% 
% % Lengths
% syms L1 L2 L7 L8 al L11 L21 L31 L32 L41 L51 real
% L = [L1,L2,L7,L8,al];
% p = [L1,L2,L7,L8,al,L11,L21,L31,L32,L41,L51];
% 
% % Inertia tensors
% syms I111 I112 I113 I122 I123 I133 real
% I1 = [I111, I112, I113;
%     I112, I122, I123;
%     I113, I123, I133];
% 
% syms I211 I212 I213 I222 I223 I233 real
% I2 = [I211, I212, I213;
%     I212, I222, I223;
%     I213, I223, I233];
% 
% syms I311 I312 I313 I322 I323 I333 real
% I3 = [I311, I312, I313;
%     I312, I322, I323;
%     I313, I323, I333];
% 
% syms I411 I412 I413 I422 I423 I433 real
% I4 = [I411, I412, I413;
%     I412, I422, I423;
%     I413, I423, I433];
% 
% % Joint Positions
% u(1) = q1;
% u(2) = q2;
% u(3) = q3;
% u(4) = q4;
% 
% % Joint Velocities
% u(5) = qp1;
% u(6) = qp2;
% u(7) = qp3;
% u(8) = qp4;
% 
% % Gravity Vector
% u(9) = gz;
% 
% % Viscous Friction
% u(10) = Beta_11;
% u(11) = Beta_22;
% u(12) = Beta_33;
% u(13) = Beta_44;
% 
% Beta(1,1)=u(10);
% Beta(2,2)=u(11);
% Beta(3,3)=u(12);
% Beta(4,4)=u(13);
% 
% % Time
% u(14) = t;
% 
% Q = u(1:4);
% 
% [HTcm_0,~] = getAbsoluteHTcm_abbIRB4(Q,p, T0_W);
% [Jcm1,Jcm2,Jcm3,Jcm4] = Jcm_abbIRB4(Q,p);
% 
% Rcm1 = HTcm_0{2}(1:3,1:3);
% Rcm2 = HTcm_0{3}(1:3,1:3);
% Rcm3 = HTcm_0{4}(1:3,1:3);
% Rcm4 = HTcm_0{5}(1:3,1:3);
% 
% 
% M = (massMatrix(m1,Jcm1,I1,Rcm1)) + (massMatrix(m2,Jcm2,I2,Rcm2)) + (massMatrix(m3,Jcm3,I3,Rcm3)) + (massMatrix(m4,Jcm4,I4,Rcm4));
% M = simplify(M);
% clc
% 
% % Display all elements of the matrix M
% disp(['M(1,1) = ', char(M(1,1))]);
% disp(['M(1,2) = ', char(M(1,2))]);
% disp(['M(1,3) = ', char(M(1,3))]);
% disp(['M(1,4) = ', char(M(1,4))]);
% disp(['M(2,1) = ', char(M(2,1))]);
% disp(['M(2,2) = ', char(M(2,2))]);
% disp(['M(2,3) = ', char(M(2,3))]);
% disp(['M(2,4) = ', char(M(2,4))]);
% disp(['M(3,1) = ', char(M(3,1))]);
% disp(['M(3,2) = ', char(M(3,2))]);
% disp(['M(3,3) = ', char(M(3,3))]);
% disp(['M(3,4) = ', char(M(3,4))]);
% disp(['M(4,1) = ', char(M(4,1))]);
% disp(['M(4,2) = ', char(M(4,2))]);
% disp(['M(4,3) = ', char(M(4,3))]);
% disp(['M(4,4) = ', char(M(4,4))]);
% 
% %% Coriolis matrix
% Qp = u(5:8);
% C = coriolisMatrix(Q,Qp,M);
% C = simplify(C);
% 
% % Display all elements of the matrix C
% disp(['C(1,1) = ', char(C(1,1))]);
% disp(['C(1,2) = ', char(C(1,2))]);
% disp(['C(1,3) = ', char(C(1,3))]);
% disp(['C(1,4) = ', char(C(1,4))]);
% disp(['C(2,1) = ', char(C(2,1))]);
% disp(['C(2,2) = ', char(C(2,2))]);
% disp(['C(2,3) = ', char(C(2,3))]);
% disp(['C(2,4) = ', char(C(2,4))]);
% disp(['C(3,1) = ', char(C(3,1))]);
% disp(['C(3,2) = ', char(C(3,2))]);
% disp(['C(3,3) = ', char(C(3,3))]);
% disp(['C(3,4) = ', char(C(3,4))]);
% disp(['C(4,1) = ', char(C(4,1))]);
% disp(['C(4,2) = ', char(C(4,2))]);
% disp(['C(4,3) = ', char(C(4,3))]);
% disp(['C(4,4) = ', char(C(4,4))]);
% 
% %% G-vector
% tcm1 = HTcm_0{2}(1:3,4);
% tcm2 = HTcm_0{3}(1:3,4);
% tcm3 = HTcm_0{4}(1:3,4);
% tcm4 = HTcm_0{5}(1:3,4);
% g0 = [0 0 gz];
% 
% P = m1*g0*tcm1+m2*g0*tcm2+m3*g0*tcm3+m4*g0*tcm4;
% 
% G = [diff(P,q1),diff(P,q2),diff(P,q3),diff(P,q4)].';
% disp(['G(1,1) = ', char(G(1))]);
% disp(['G(2,1) = ', char(G(2))]);
% disp(['G(3,1) = ', char(G(3))]);
% disp(['G(4,1) = ', char(G(4))]);
% 
% % %% Qpp (way too complex with symbolic variables)
% % Tao = zeros(4,1)
% % Qpp = M\(tao-C*Qp.' - G - Beta*Qp.')
% 
% %% Custom functions
% 
% function M = massMatrix(m,J,I,R)
%     Jv = J(1:3,:);
%     Jw = J(4:6,:);
%     M = simplify(m*(Jv.')*Jv + (Jw.')*R*I*(R.')*Jw);
% end
% 
% 
% function C = coriolisMatrix(q, q_dot, massMatrix)
% n = length(q);  % Number of joints
% C = sym(zeros(n, n));  % Initialize Coriolis matrix as symbolic zeros
% 
% for k = 1:n
%     for j = 1:n
%         c_kj = 0;
%         for i = 1:n
%             c_kj = c_kj + 0.5 * (diff(massMatrix(k, j), q(i)) + ...
%                 diff(massMatrix(k, i), q(j)) - diff(massMatrix(i, j), q(k))) * q_dot(i);
%         end
%         C(k, j) = c_kj;
%     end
% end
% end
% 
% end
% 
% % syms I111 I112 I113 I122 I123 I133 real
% % syms I211 I212 I213 I222 I223 I233 real
% % 
% % syms I311 I312 I313 I322 I323 I333 real
% % syms I411 I412 I413 I422 I423 I433 real
% % syms m1 m2 m3 m2 real
% % 
% % syms q1 q2 q3 q4 qp1 qp2 qp3 qp4 gz real
% % syms Beta_11 Beta_22 Beta_33 Beta_44 real
% % syms t real
% % syms m1 m2 m3 m4 real
% % 
% % 
% % 
% % % Inertia Tensor
% % I{1}=[I111 I112 I113;I112 I122 I123;I113 I123 I133];
% % I{2}=[I211 I212 I213;I212 I222 I223;I213 I223 I233];
% % 
% % I{3}=[I311 I312 I313;I312 I322 I323;I313 I323 I333];
% % I{4}=[I411 I412 I413;I412 I422 I423;I413 I423 I433];
% % 
% % m={m1,m1,m1,m2};
% % 
% % M=0;
% % 
% % for i=1:n
% %     Jv=Jcm_0{i}(1:3,:); % position part of jacobian
% %     Jw=Jcm_0{i}(4:6,:); % orientation part of jacobian
% %     Mt=m{i}*Jv'*Jv+Jw'*Rc{i}*I{i}*Rc{i}'*Jw; % M_i(q)
% %     %disp(Mt)
% %     M=M+Mt; % sumation
% % end
% % 
% % 
% % % syms dq1 dq2 dq3 dq4 C real
% % 
% % % 
% % Mp{1}=diff(M,q1);
% % Mp{2}=diff(M,q2);
% % Mp{3}=diff(M,q3);
% % Mp{4}=diff(M,q4);
% % 
% % qp={dq1,dq2,dq3,dq4};
% % 
% % 
% % for k=1:n
% %     for j=1:n
% %         c=0;
% %         for i=1:n
% %             cr=(Mp{i}(k,j)+Mp{j}(k,i)-Mp{k}(i,j))*qp{i};
% %             c=c+cr;
% %         end
% %         C(k,j)=(1/2)*(c);
% %     end
% % end
% % 
% % 
% % % syms G P g real
% % 
% % % gravitational acceleration
% % vg=[0;0;-g];
% % 
% % % Potential Energy
% % P=0;
% % for i=1:n
% %     tc{i+1};
% %     p=(m{i}*vg'*tc{i+1});
% %     P=P+p;
% % end
% % P=(P);
% % % fprintf('P');
% % 
% % % disp(P)
% % % Gravitational force vector
% % q={q1,q2,q3,q4};
% % for i=1:n
% %     G(i,1)=(diff(P,q{i}));
% % end
% % % fprintf('G');
% % % disp(G)
% % end
% % 
% % 
% function H = dhHT(u)
% %DHHT Calculates the HT from DH parameters (Distal)
% 
% theta=u(1);
% d=u(2);
% a=u(3);
% alpha=u(4);
% 
% %HT Rotation in z (theta)
% Hrz=[cos(theta) -sin(theta) 0 0;
%      sin(theta)  cos(theta) 0 0;
%      0           0          1 0;
%      0           0          0 1];
% 
% % HT Translation in z (d)
% Htz=[1 0 0 0;
%      0 1 0 0;
%      0 0 1 d;
%      0 0 0 1];
% 
% %HT Translation in x (a)
% Htx=[1 0 0 a;
%      0 1 0 0;
%      0 0 1 0;
%      0 0 0 1];
% 
% %HT Rotation in x (alpha)
% Hrx=[1 0           0          0;
%      0 cos(alpha) -sin(alpha) 0;
%      0 sin(alpha)  cos(alpha) 0;
%      0 0           0          1];
% 
% % DH Homogeneous Transformation
% H=(Hrz*Htz*Htx*Hrx);
% 
% end
% % 
% % 
% function J = Jac(t,z,p,j,joints)
% %JAC Computes Geometric JAcobian
% %   This function computes the geometric jacobian
% %Input:
% % t: List of link positions, including t0_0 (n+1)
% % z: list of axis of motion, including z0_0 (n+1)
% % p: target point in link id
% % j: link id > 0 <= n
% % joints: List of joint types (n)
% 
% syms J real
% 
% z3D=[0;0;0];
% 
% % DOFs
% n=numel(t)-1;
% 
% J(6,n)=0;
% 
% % n Columns
% for i=1:n
%     % Check if the joint contributes to changes in velocities
%     if(i<=j)
%         %Check the type of joint
%         if joints{i}=='R'
%             % Revolute Joint
%             v=(cross(z{i},(p-t{i})));
%             w=z{i};
%         else
%             % Prismatic Joint
%             v=z{i};
%             w=z3D;
%         end
%     else
%         %NO contribution to the velocity
%         v=z3D;
%         w=z3D;
%     end
%     J(1:6,i)=[v;w];
% end
% 
% 
% 
% end

