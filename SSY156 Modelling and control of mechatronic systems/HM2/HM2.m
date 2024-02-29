clear, clc;

% syms q1 q2 q3 l1 l2 l3 l4 l5 l6 l7 l8 l9 l10 l11 beta halv_pi real

q1 =0
q2= 0
q3=0
l1= 0.1
l2=0.3 
l3=0.23 
l4 = 0.42
l5=0.1
l6=0.35
l7=0.07
l8=0.5
l9=l6/2 
l10 = l2/2
l11=l4/2
beta = 2/sqrt(2);


H0_w = [
    0   0     1   0;
    0   -1    0   0;
    1   0     0   0;
    0   0     0   1
]

H0_0 = eye(4)

% Relative homogenous matrices for the joints

H1_0 = dhHT(beta, q1 +l1, 0, -pi/2)

Hv1_1 = dhHT(-pi/2, q2, 0, -pi/2 - beta )

H2_v1 = dhHT(-pi/2, l2, 0, -pi/2)

Hv2_2 = dhHT(-pi/2 + q3, l5, 0, -pi/2)

He_v2 = dhHT(pi, l3+l4, 0, 0)

% In the question Hef_3 
Hef_2 = Hv2_2*He_v2

% Absolute matrices fot the joints wrt base

T1_0 = H1_0

T2_0 = H1_0*Hv1_1*H2_v1

Tef_0 = H1_0*Hv1_1*H2_v1*Hv2_2*He_v2

% Absolute matrices wrt world

T1_w = H0_w*T1_0

T2_w = H0_w*T2_0

% Hef_w
T3_w = H0_w*Tef_0

% center of masses
% Relative homogenous matrices for the center of masses

Mcm1_1  = dhHT(0, q2+2*l9/sqrt(2), 0, 0)

Mv1_1   = dhHT(-pi/2, q2, 0, -pi/2- beta)

Mcm2_v1 = dhHT(0, l10, 0, 0)

Mcm3_2  = dhHT(q3, 0, l3+l11, 0)

% homogenous matrices for the center of masses wrt base

Tcm1_0 = T1_0 * Mcm1_1
Tcm2_0 = T1_0 * Mv1_1*Mcm2_v1
Tcm3_0 = T2_0 * Mcm3_2

% Absolute matrices wrt world

Tcm1_w = H0_w * Tcm1_0
Tcm2_w = H0_w * Tcm2_0
Tcm3_w = H0_w * Tcm3_0

% Jacobian for each link

% angular velocity for joints
z0_0 = H0_0(1:3,3);
z1_0 = T1_0(1:3,3);
z2_0 = T2_0(1:3,3);
z3_0 = Tef_0(1:3,3);

% Position for joints
t1_0 = T1_0(1:3,4); 
t2_0 = T2_0(1:3,4); 
tef_0 = Tef_0(1:3,4);

Jef_0 = [ 
        z0_0,                               z1_0,                           cross(z2_0,(tef_0-t2_0));
        zeros(3,1),                        zeros(3,1),                      z2_0                    
       ]

% Jacobian for center of mass

% Position for center of masses
tcm1_0 = Tcm1_0(1:3,4); 
tcm2_0 = Tcm2_0(1:3,4); 
tcm3_0 = Tcm3_0(1:3,4);

Jcm_0 = [ 
        z0_0,                               z1_0,                           cross(z2_0,(tef_0-tcm2_0));
        zeros(3,1),                        zeros(3,1),                      z2_0                    
       ]