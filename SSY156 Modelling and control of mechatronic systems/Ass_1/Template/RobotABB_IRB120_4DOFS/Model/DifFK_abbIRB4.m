    function Xp=DifFK_abbIRB4(q,qp,L)
%DIFFK_ABBIRB4 calculates the end-effector's linear and angular velocity
%vector wrt the robot base (link 0)
% input:
%   q: is the joint position vector 1X4
%   qp: is the joint velocity vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
% return:
%   Xp: End-effector linear and angular velocitites wrt robot base (6X1)


% Joint positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Joint velocities
qp1=qp(1);
qp2=qp(2);
qp3=qp(3);
qp4=qp(4);

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

% TODO: define the Jacobian of the ef wrt robot base (link 0) 6xn
%   Jef_0 =[L8*sin(q4 - al + pi/2)*(cos(al + q3)*sin(q1)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)*sin(q1)) - L8*cos(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2)*sin(q1) - sin(al + q3)*sin(q1)*sin(q2 - pi/2)) - L2*cos(q2 - pi/2)*sin(q1) - L7*cos(al + q3)*cos(q2 - pi/2)*sin(q1) + L7*sin(al + q3)*sin(q1)*sin(q2 - pi/2),                                                                                                                                                                                                                                                                                                                                                                                            -cos(q1)*(L2*sin(q2 - pi/2) + L7*cos(al + q3)*sin(q2 - pi/2) + L7*sin(al + q3)*cos(q2 - pi/2) + L8*cos(q4 - al + pi/2)*(cos(al + q3)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)) + L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2) - sin(al + q3)*sin(q2 - pi/2))),                                                                                                                                                                                                                                                                                                                                                        -cos(q1)*(L7*cos(al + q3)*sin(q2 - pi/2) + L7*sin(al + q3)*cos(q2 - pi/2) + L8*cos(q4 - al + pi/2)*(cos(al + q3)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)) + L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2) - sin(al + q3)*sin(q2 - pi/2))),                                                                                                                                                                                                                                                      -cos(q1)*(L8*cos(q4 - al + pi/2)*(cos(al + q3)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)) + L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2) - sin(al + q3)*sin(q2 - pi/2)));
% L8*cos(q4 - al + pi/2)*(cos(al + q3)*cos(q1)*cos(q2 - pi/2) - sin(al + q3)*cos(q1)*sin(q2 - pi/2)) - L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q1)*sin(q2 - pi/2) + sin(al + q3)*cos(q1)*cos(q2 - pi/2)) + L2*cos(q1)*cos(q2 - pi/2) - L7*sin(al + q3)*cos(q1)*sin(q2 - pi/2) + L7*cos(al + q3)*cos(q1)*cos(q2 - pi/2),                                                                                                                                                                                                                                                                                                                                                                                            -sin(q1)*(L2*sin(q2 - pi/2) + L7*cos(al + q3)*sin(q2 - pi/2) + L7*sin(al + q3)*cos(q2 - pi/2) + L8*cos(q4 - al + pi/2)*(cos(al + q3)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)) + L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2) - sin(al + q3)*sin(q2 - pi/2))),                                                                                                                                                                                                                                                                                                                                                        -sin(q1)*(L7*cos(al + q3)*sin(q2 - pi/2) + L7*sin(al + q3)*cos(q2 - pi/2) + L8*cos(q4 - al + pi/2)*(cos(al + q3)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)) + L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2) - sin(al + q3)*sin(q2 - pi/2))),                                                                                                                                                                                                                                                      -sin(q1)*(L8*cos(q4 - al + pi/2)*(cos(al + q3)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)) + L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2) - sin(al + q3)*sin(q2 - pi/2)));
%                                                                                                                                                                                                                                                                                                                     0, - cos(q1)*(L8*cos(q4 - al + pi/2)*(cos(al + q3)*cos(q1)*cos(q2 - pi/2) - sin(al + q3)*cos(q1)*sin(q2 - pi/2)) - L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q1)*sin(q2 - pi/2) + sin(al + q3)*cos(q1)*cos(q2 - pi/2)) + L2*cos(q1)*cos(q2 - pi/2) - L7*sin(al + q3)*cos(q1)*sin(q2 - pi/2) + L7*cos(al + q3)*cos(q1)*cos(q2 - pi/2)) - sin(q1)*(L8*cos(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2)*sin(q1) - sin(al + q3)*sin(q1)*sin(q2 - pi/2)) - L8*sin(q4 - al + pi/2)*(cos(al + q3)*sin(q1)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)*sin(q1)) + L2*cos(q2 - pi/2)*sin(q1) + L7*cos(al + q3)*cos(q2 - pi/2)*sin(q1) - L7*sin(al + q3)*sin(q1)*sin(q2 - pi/2)), - cos(q1)*(L8*cos(q4 - al + pi/2)*(cos(al + q3)*cos(q1)*cos(q2 - pi/2) - sin(al + q3)*cos(q1)*sin(q2 - pi/2)) - L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q1)*sin(q2 - pi/2) + sin(al + q3)*cos(q1)*cos(q2 - pi/2)) - L7*sin(al + q3)*cos(q1)*sin(q2 - pi/2) + L7*cos(al + q3)*cos(q1)*cos(q2 - pi/2)) - sin(q1)*(L8*cos(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2)*sin(q1) - sin(al + q3)*sin(q1)*sin(q2 - pi/2)) - L8*sin(q4 - al + pi/2)*(cos(al + q3)*sin(q1)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)*sin(q1)) + L7*cos(al + q3)*cos(q2 - pi/2)*sin(q1) - L7*sin(al + q3)*sin(q1)*sin(q2 - pi/2)), - sin(q1)*(L8*cos(q4 - al + pi/2)*(cos(al + q3)*cos(q2 - pi/2)*sin(q1) - sin(al + q3)*sin(q1)*sin(q2 - pi/2)) - L8*sin(q4 - al + pi/2)*(cos(al + q3)*sin(q1)*sin(q2 - pi/2) + sin(al + q3)*cos(q2 - pi/2)*sin(q1))) - cos(q1)*(L8*cos(q4 - al + pi/2)*(cos(al + q3)*cos(q1)*cos(q2 - pi/2) - sin(al + q3)*cos(q1)*sin(q2 - pi/2)) - L8*sin(q4 - al + pi/2)*(cos(al + q3)*cos(q1)*sin(q2 - pi/2) + sin(al + q3)*cos(q1)*cos(q2 - pi/2)));
%                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                -sin(q1);
%                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                 cos(q1);
%                                                                                                                                                                                                                                                                                                                     1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                       0];
    Jef_0=J_EF_abbIRB4(q,L);
%  %TODO: Calcualte the linear and angular velocity vector of the ef wrt 0
   Xp= Jef_0 * [qp1;qp2; qp3;qp4]; 