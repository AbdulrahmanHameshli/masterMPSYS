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


%% Inertia Matrix
%TODO: define the inertia Matrix.
M(1,1)=...;
M(1,2)=...;
M(1,3)=...;
M(1,4)=...;
M(2,1)=...;
M(2,2)=...;
M(2,3)=...;
M(2,4)=...;
M(3,1)=...;
M(3,2)=...;
M(3,3)=...;
M(3,4)=...;
M(4,1)=...;
M(4,2)=...;
M(4,3)=...;
M(4,4)=...;


%% Centripetal and Coriolis Matrix
%TODO: define the C matrix
C(1,1)=...;
C(1,2)=...;
C(1,3)=...;
C(1,4)=...;
C(2,1)=...;
C(2,2)=...;
C(2,3)=...;
C(2,4)=...;
C(3,1)=...;
C(3,2)=...;
C(3,3)=...;
C(3,4)=...;
C(4,1)=...;
C(4,2)=...;
C(4,3)=...;
C(4,4)=...;


%% Gravitational Torques Vector
%TODO: define the G vector
G(1,1)=...;
G(2,1)=...;
G(3,1)=...;
G(4,1)=...;

%TODO: define the external torque (4x1) to simulate the natural dynamics of the
%robot
Tau=...;

%TODO: compute the direct dynamics using M, C, G, B, and Tau
Qpp=...;
