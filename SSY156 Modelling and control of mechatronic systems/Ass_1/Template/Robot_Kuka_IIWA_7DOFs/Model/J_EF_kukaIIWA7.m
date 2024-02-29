function Jef_0=J_EF_kukaIIWA7(q,L)
%J_EF_KUKAIIWA7 calculates the geometric Jacobian of the end-effector wrt the
%robot base (link 0)
% input:
%   q: is the joint position vector 1X7
%   L: is the kinematic parameter array (see kukaIIWA7_params.m)
% return:
%   Jef_0: End-effector Jacobian wrt robot base (6Xn)

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

c1=cos(q1);
c2=cos(q2);
c3=cos(q3);
c4=cos(q4);
c5=cos(q5);
c6=cos(q6);

s12=s1*s2;
s13=s1*s3;
s14=s1*s4;
s15=s1*s5;
s16=s1*s6;

s23=s2*s3;
s24=s2*s4;

s34=s3*s4;
s35=s3*s5;
s36=s3*s6;

s45=s4*s5;

s56=s5*s6;

c12=c1*c2;
c13=c1*c3;
c14=c1*c4;

c23=c2*c3;
c24=c2*c4;
c25=c2*c5;
c26=c2*c6;

c34=c3*c4;
c35=c3*c5;
c36=c3*c6;

c45=c4*c5;
c46=c4*c6;

c56=c5*c6;

c123=c12*c3;
c12s3=c12*s3;
c1s23=c1*s24;
c14s2=c14*s2;
c23s1=c23*s1;
c34s2=c34*s2;
c4s12=c4*s12;
c2s13=c2*s13;
s124=s12*s4;
c46s2=c46*s2;
c236s4=c23*c6*s4;
c3s24=c3*s24;
c5s246=c5*s24*s6;
c2345s6=c23*c45*s6;
c6s34=c6*s34;
c26s4=c26*s4;
c245s6=c24*c5*s6;
c5s23=c5*s23;
c2s45=c2*s45;
c24s6=c24*s6;
c256s4=c25*c6*s4;
c6s235=c6*s23*s5;
c3456s2=c34*c56*s2;
c136s4=c13*c6*s4;
c13s4=c13*s4;
c1s356=c1*s35*s6;
c26s134=c26*s13*s4;
c1345s6=c13*c45*s6;
c36s14=c36*s14;
s1356=s13*s56;
c245s136=c24*c5*s13*s6;
c345s16=c34*c5*s16;
c35s1=c35*s1;
c125s3=c12*c5*s3;
c4s135=c4*s13*s5;
c25s13=c25*s13;
c135=c13*c5;
c14s35=c14*s35;
c234s15=c23*c4*s15;
s135=s23*s5;


H1_0 = dhHT(q1, L1, 0, -pi/2);

H2_1 = dhHT(q2, 0, 0, pi/2);

H3_2 = dhHT(q3, L2, 0, pi/2);

H4_3 = dhHT(q4, 0, 0, -pi/2);

H5_4 = dhHT(q5, L3, 0, -pi/2);

H6_5 = dhHT(q6, 0, 0, pi/2);

H7_6 = dhHT(q7, L4, 0, 0);
% Evaluate the result: Is the Transformation correct?



%% Absolute Ht
H1_0 = H1_0;
H2_0 = H1_0*H2_1;
H3_0 = H1_0*H2_1*H3_2;
H4_0 = H1_0*H2_1*H3_2*H4_3;
H5_0 = H1_0*H2_1*H3_2*H4_3 * H5_4;
H6_0 =H1_0*H2_1*H3_2*H4_3*H5_4*H6_5;
Hef_0 =H1_0*H2_1*H3_2*H4_3*H5_4*H6_5*H7_6;
%%Jacobian:
        %Revolut joints angular veloity;
        z1_0 = H1_0(1:3,3); 
        z2_0 = H2_0(1:3,3); 
        z3_0 = H3_0(1:3,3);
        z4_0 = H4_0(1:3,3);
        z5_0 = H5_0(1:3,3);
        z6_0= H6_0(1:3,3);
        zef_0 =Hef_0(1:3,3);
        %Lineiar velocity R joint: velocity  = Angular_velocity * Radius =
        %theta * r
        t1_0 = H1_0(1:3,4); 
        t2_0 = H2_0(1:3,4); 
        t3_0 = H3_0(1:3,4);
        t4_0 = H4_0(1:3,4); 
        t5_0 = H5_0(1:3,4); 
        t6_0 = H6_0(1:3,4); 
        tef_0 =Hef_0(1:3,4);

% TODO: define the Jacobian of the ef wrt robot base (link 0) 6xn

Jef_0 = [ cross([0;0;1],(tef_0-[0;0;0])), cross(z1_0,(tef_0-t1_0)),cross(z2_0,(tef_0-t2_0)),cross(z3_0,(tef_0-t3_0)) ,cross(z4_0,(tef_0-t4_0)) ,cross(z5_0,(tef_0-t5_0)) ,cross(z6_0,(tef_0-t6_0));
           [0;0;1],             z1_0,                   z2_0,                       z3_0,    z4_0,                      z5_0,                       z6_0];

end