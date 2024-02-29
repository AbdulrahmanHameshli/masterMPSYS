function Jef_0=J_EF_abbIRB4(q,L)
%J_EF_ABBIRB4 calculates the geometric Jacobian of the end-effector wrt the
%robot base (link 0)
% input:
%   q: is the joint position vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
% return:
%   Jef_0: End-effector Jacobian wrt robot base (6Xn)


% Joint Positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic paramters
L1=L(1);
L2=L(2);
L7=L(3);
L8=L(4);
al=L(5);

% Common substitutions
c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

% TODO: Calculate the Jacobian of the end-effector wrt link 0 (6xn)
%% Relative First link
H1_0 = dhHT(q1,L1,0,-pi/2)
% Evaluate the result: Is the Transformation correct?
%% Relative Second Link
H2_1=dhHT(q2-(pi/2),0,L2,0)
% Evaluate the result: Is the Transformation correct?
%% Relative Third Link
H3_2=dhHT(q3+al,0,L7,0)
%% Relative Third Link
H4_3=dhHT(q4-al+pi/2,0,L8,0)
% Hef_4=dhHT(pi/2,0,0,pi/2)
% Evaluate the result: Is the Transformation correct?

%% Absolute Ht
H2_0 = H1_0*H2_1;
H3_0 = H1_0*H2_1*H3_2;
Hef_0 = H1_0*H2_1*H3_2*H4_3;

%%Jacobian:
        %Revolut joints angular veloity;
        z1_0 = H1_0(1:3,3); 
        z2_0 = H2_0(1:3,3); 
        z3_0 = H3_0(1:3,3); 
        zef_0 =Hef_0(1:3,3);
        %Lineiar velocity R joint: velocity  = Angular_velocity * Radius =
        %theta * r
        t1_0 = H1_0(1:3,4); 
        t2_0 = H2_0(1:3,4); 
        t3_0 = H3_0(1:3,4); 
        tef_0 =Hef_0(1:3,4);

Jef_0 = [ cross([0;0;1],(tef_0-[0;0;0])), cross(z1_0,(tef_0-t1_0)),cross(z2_0,(tef_0-t2_0)),cross(z3_0,(tef_0-t3_0));
           [0;0;1],             z1_0,                   z2_0,                       z3_0];
end