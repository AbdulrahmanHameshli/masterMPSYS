%% Example robot RRP  Lecture 4 (SSY156_Lecture04_RobotKinematics.pdf, pp 44)

%% Symbolic variables
syms q1 q2 q3 q4 L1 L2 L3 L4 L6 L7 L8 al real

% Use pi as a symbol
pi=sym(pi);

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
H2_0 = H1_0*H2_1
H3_0 = H1_0*H2_1*H3_2
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

% Evaluate the result: Is the Transformation correct? Use values, e.g.

q1=pi/2;
q2=1;
q3=2;

% H3_0=eval(H3_0);

% Is the result correct?


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
