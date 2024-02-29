function [Jcm1,Jcm2,Jcm3,Jcm4] = Jcm_abbIRB4(q,L)
%JCM CM geometric Jacobians
%   q: Joint position vector 4x1
%   L: Dynamic parameters, see abbIRB4_dyn_params.m

% Joint Positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic and dynamic parameters
L2=L(2);
L7=L(3);
al=L(5);
L21=L(7);
L41=L(10);
L51=L(11);

% Common substitutions
c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

% Jacobians CM
%TODO: define the geometric Jacobians for each CM
Jcm1=[....];
				
Jcm2=[....];
				
Jcm3=[....];

Jcm4=[....];


end

