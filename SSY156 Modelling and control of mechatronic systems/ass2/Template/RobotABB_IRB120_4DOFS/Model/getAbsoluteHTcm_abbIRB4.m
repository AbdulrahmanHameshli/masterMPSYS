function [HTcm_0, HTcm_W] = getAbsoluteHTcm_abbIRB4(q,L, T0_W)
%GETABSOLUTEHTCM calculates the absolute Homogeneous transformations of
% the cms wrt the robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X4
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HTcm_0: List with all the absolute CM HT (4x4) wrt robot base (0)
%   HTcm_W: List with all the absolute CM HT (4x4) wrt wcf (w)

% Joint postion

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic Paramters
%    1  2   3  4  5  6   7   8   9  10  11
% p=[L1,L2,L7,L8,al,L11,L21,L31,L32,L41,L51];

L1=L(1);
L2=L(2);
L7=L(3);
al=L(5);
L11=L(6);
L21=L(7);
L41=L(10);
L51=L(11);

% Common substitutions

c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

s234=sin(q2 + q3 + q4);
c234=cos(q2 + q3 + q4);


% Absolute Homogeneous Transformations
% TODO: Define the absolute HT wrt robot base (link 0)

aHd = AbsoluteDHforCM(q)
    Tcm1_0 = aHd{1}
    Tcm2_0 = aHd{2} 
    Tcm3_0 = aHd{3}
    Tcm4_0 = aHd{4}
    
    % List with all the HT wrt to link 0, starting with H0_0
    HTcm_0 = {eye(4), Tcm1_0, Tcm2_0, Tcm3_0, Tcm4_0};
    
    %TODO: Create a list with all the HT wrt to the wcf (w), starting with
    %Hw_w. In total, HT_W should have 6 elements Hw_w, H0_w, ..., Hef_w.
HTcm_W = {eye(4), T0_W, T0_W*Tcm1_0, T0_W*Tcm2_0, T0_W*Tcm3_0,T0_W*Tcm4_0};


end


function aHd = AbsoluteDHforCM(q)

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
n = 4
L1 = 0.29;
    L2 = 0.27;
    L3 = 0.07;
    L4 = 0.302;
    L5 = 0.072;
    L6 = 0.1;
L7=0.31;
L8=0.172;
al=1.343;

    L11 = L1/2;
    L21 = L2/2;
    L31 = L3/2;
    L32 = L4/2;
    L41 = L8/2;
L51=0.155;

H1_0 = dhHT([q1,L1, 0, -pi/2]);
H2_1 = dhHT([q2-pi/2, 0,L2, 0]);
H3_2 = dhHT([q3+al, 0, L7, 0]);
Hv2_v1 = dhHT([q4-al+pi/2,0,L8+L6,0]);
Hef_v2 = dhHT([pi/2,0,0,pi/2]);
% Homogeneous Transformations links
H{1} = H1_0;
H{2} = H2_1;
H{3} = H3_2;
H{4} = Hv2_v1;

% Homogeneous Transformations cm

Hcm1_0 = dhHT([q1, L11, 0, 0]);
Hcm2_1  = dhHT([q2-pi/2, 0, L21, 0]);
Hcm3_2  = dhHT([q3+al, 0, L51, 0]);
Hcm4_3  = dhHT([q4-al+pi/2, 0, L41, 0]);

Hc{1} = Hcm1_0;
Hc{2} = Hcm2_1;
Hc{3} = Hcm3_2;
Hc{4} = Hcm4_3;

% aH{1}=H1_0;
% aH{2}=H1_0* H2_1;
% aH{3}=H1_0* H2_1 * H3_2;
% aH{4}=H1_0* H2_1 * H3_2 * H{4};
% 
% aHc{1} = Hc{1};
% aHc{2} = aH{1}*Hc{2};
% aHc{3} = aH{2}*Hc{3};
% aHc{4} = aH{3}*Hc{4};

aH{1}=H{1};


for i=2:n
    aH{i}=aH{i-1}*H{i};
end

aHc{1}=Hc{1};
for i=2:n
    aHc{i}=(aH{i-1}*Hc{i});
end


aHd = aHc

end


function H = dhHT(u)
%DHHT Calculates the HT from DH parameters (Distal)

theta=u(1);
d=u(2);
a=u(3);
alpha=u(4);

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
H=(Hrz*Htz*Htx*Hrx);

end
