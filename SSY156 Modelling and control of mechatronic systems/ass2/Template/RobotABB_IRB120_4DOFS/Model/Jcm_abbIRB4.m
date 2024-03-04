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
 jcm = GetJcm_0(q)
    Jcm1=jcm{1}
				    
    Jcm2=jcm{2}
				    
    Jcm3=jcm{3}
    
    Jcm4=jcm{4}

end


function J = GetJcm_0(q)


joints={'R','R','R','R'};
n = 4
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

aH{1}=H{1};


for i=2:n
    aH{i}=aH{i-1}*H{i};
end

aHc{1}=Hc{1};
for i=2:n
    aHc{i}=(aH{i-1}*Hc{i});
end


aHd = aHc



syms z3d real

z{1}=[0;0;1];
t{1}=[0;0;0];

for i=1:n
    
    % axis of motion third 3D column vector absolute H
    z{i+1}=aH{i}(1:3,3);
    % link position vector fourth 3D column vector absolute H
    t{i+1}=aH{i}(1:3,4);

    % cm position vector fourth 3D column vector absolute H
    tc{i+1}=aHc{i}(1:3,4);

    % Rotiation Matrix CM
    Rc{i}=aHc{i}(1:3,1:3);

    % fprintf('z%i = ', i);
    % disp(z{i+1})
    % fprintf('p%i = ', i);
    % disp(t{i+1})
    % fprintf('p_cm%i = ', i);
    % disp(tc{i+1})
    % fprintf('Rc%i = ', i);
    % disp(Rc{i})
end

syms Jef_0 real

Jef_0 = Jac(t,z,t{end},n,joints);


% Jval=([diff(t{end},q1), diff(t{end},q2), diff(t{end},q3), diff(t{end},q4)]);

% DJ=(Jef_0(1:3,:)-Jval);

syms Jcm_0 J real

Jcm_0={'','','',''};

for j=1:n
    Jcm_0{j}=Jac(t,z,tc{j+1},j,joints);
    % fprintf('J_cm for link %i ', j);
    % disp(Jcm_0{j})

    % Validation
    % Jval=simplify([diff(tc{j+1},q1), diff(tc{j+1},q2)]);
    % DJcm=simplify(Jcm_0{j}(1:3,:)-Jval)
end

J = Jcm_0


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




function J = Jac(t,z,p,j,joints)
%JAC Computes Geometric JAcobian
%   This function computes the geometric jacobian
%Input:
% t: List of link positions, including t0_0 (n+1)
% z: list of axis of motion, including z0_0 (n+1)
% p: target point in link id
% j: link id > 0 <= n
% joints: List of joint types (n)

syms     real

z3D=[0;0;0];

% DOFs
n=numel(t)-1;

J(6,n)=0;

% n Columns
for i=1:n
    % Check if the joint contributes to changes in velocities
    if(i<=j)
        %Check the type of joint
        if joints{i}=='R'
            % Revolute Joint
            v=(cross(z{i},(p-t{i})));
            w=z{i};
        else
            % Prismatic Joint
            v=z{i};
            w=z3D;
        end
    else
        %NO contribution to the velocity
        v=z3D;
        w=z3D;
    end
    J(1:6,i)=[v;w];
end



end

