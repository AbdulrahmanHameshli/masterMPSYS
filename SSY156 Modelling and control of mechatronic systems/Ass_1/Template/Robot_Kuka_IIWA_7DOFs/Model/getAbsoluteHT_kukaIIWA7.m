function [HT_0, HT_W] = getAbsoluteHT_kukaIIWA7(q,L, T0_W)
%GETABSOLUTEHT_KUKAIIWA7 calculates the absolute Homogeneous transformations wrt the
%robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X7
%   L: is the kinematic parameter array (see abbIRB4_params.m)
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HT_0: Cell array with all the absolute HT (4x4) wrt robot base
%   HT_W: Cell array with all the absolute HT (4x4) wrt wcf

% Joint positions

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);
q7=q(7);

% Kinematic parameters
L1=L(1);
L2=L(2);
L3=L(3);
L4=L(4);

% Common Substitutions
s1=sin(q1);
s2=sin(q2);
s3=sin(q3);
s4=sin(q4);
s5=sin(q5);
s6=sin(q6);
s7=sin(q7);

c1=cos(q1);
c2=cos(q2);
c3=cos(q3);
c4=cos(q4);
c5=cos(q5);
c6=cos(q6);
c7=cos(q7);

s12=s1*s2;
s13=s1*s3;

s23=s2*s3;
s24=s2*s4;

c12=c1*c2;
c13=c1*c3;
c14=c1*c4;

c23=c2*c3;
c24=c2*c4;

c34=c3*c4;


c123=c12*c3;
c12s3=c12*s3;
c1s23=c1*s24;
c14s2=c14*s2;
c23s1=c23*s1;
c34s2=c34*s2;
c4s12=c4*s12;
c2s13=c2*s13;
s124=s12*s4;
c3s24=c3*s24;
c5s23=c5*s23;
s135=s23*s5;

% Absolute Homogeneous Transformations
% TODO: Define the absolute HT wrt robot base (link 0)


H1_0 = dhHT(q1, L1, 0, -pi/2)

H2_1 = dhHT(q2, 0, 0, pi/2)

H3_2 = dhHT(q3, L2, 0, pi/2)

H4_3 = dhHT(q4, 0, 0, -pi/2)

H5_4 = dhHT(q5, L3, 0, -pi/2)

H6_5 = dhHT(q6, 0, 0, pi/2)

H7_6 = dhHT(q7, L4, 0, 0)
% Evaluate the result: Is the Transformation correct?



%% Absolute Ht
H2_0 = H1_0*H2_1
H3_0 = H1_0*H2_1*H3_2
H4_0 = H1_0*H2_1*H3_2*H4_3;
H5_0 = H1_0*H2_1*H3_2*H4_3 * H5_4
H6_0 =H1_0*H2_1*H3_2*H4_3*H5_4*H6_5
Hef_0 =H1_0*H2_1*H3_2*H4_3*H5_4*H6_5*H7_6

T1_0=H1_0;

T2_0=H2_0;

T3_0=H3_0;

T4_0=H4_0;

T5_0=H5_0;

T6_0=H6_0;

T7_0=Hef_0;


% List with all the HT wrt to link 0, starting with H0_0
HT_0={eye(4),T1_0, T2_0, T3_0, T4_0,T5_0, T6_0, T7_0};

%TODO: Create a list with all the HT wrt to the wcf (w), starting with
%Hw_w. In total, HT_W should have 9 elements Hw_w, H0_w, ..., Hef_w.
HT_W={eye(4),T0_W,T0_W*T1_0,T0_W* T2_0,T0_W* T3_0,T0_W* T4_0,T0_W*T5_0,T0_W* T6_0,T0_W* T7_0};

end


function H = dhHT(theta,d,a,alpha)
%DHHT Calculates the HT from DH parameters (Distal)

%HT Rotation in z (theta)
Hrz=[cos(theta) -sin(theta) 0 0;
     sin(theta)  cos(theta) 0 0;
     0           0          1 0;
     0           0          0 1];

%HT Translation in z (d)
Htz=[1 0 0 0;
     0 1 0 0;
     0 0 1 d;
     0 0 0 1];
     
%HT Translation in x (a)
Htx=[1 0 0 a;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
     
%HT Rotation in x (alpha)
Hrx=[1 0           0          0;
     0 cos(alpha) -sin(alpha) 0;
     0 sin(alpha)  cos(alpha) 0;
     0 0           0          1];

% DH Homogeneous Transformation
H=Hrz*Htz*Htx*Hrx;

end