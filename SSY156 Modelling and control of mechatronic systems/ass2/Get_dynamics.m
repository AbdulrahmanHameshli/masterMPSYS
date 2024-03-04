function [M, C, G] = Get_dynamics(q)

joints={'R','R','R','R'};
n = 4
% q1=q(1);
% q2=q(2);
% q3=q(3);
% q4=q(4);
q1=0;
q2=0;
q3=0;
q4=0;
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


syms I111 I112 I113 I122 I123 I133 real
syms I211 I212 I213 I222 I223 I233 real

syms I311 I312 I313 I322 I323 I333 real
syms I411 I412 I413 I422 I423 I433 real
syms m1 m1 m1 m2


% Inertia Tensor
I{1}=[I111 I112 I113;I112 I122 I123;I113 I123 I133];
I{2}=[I211 I212 I213;I212 I222 I223;I213 I223 I233];

I{3}=[I311 I312 I313;I312 I322 I323;I313 I323 I333];
I{4}=[I411 I412 I413;I412 I422 I423;I413 I423 I433];

m={m1,m1,m1,m2};

M=0;

for i=1:n
    Jv=Jcm_0{i}(1:3,:); % position part of jacobian
    Jw=Jcm_0{i}(4:6,:); % orientation part of jacobian
    Mt=m{i}*Jv'*Jv+Jw'*Rc{i}*I{i}*Rc{i}'*Jw; % M_i(q)
    %disp(Mt)
    M=M+Mt; % sumation
end


syms dq1 dq2 dq3 dq4 C real

% 
Mp{1}=diff(M,q1);
Mp{2}=diff(M,q2);
Mp{3}=diff(M,q3);
Mp{4}=diff(M,q4);

qp={dq1,dq2,dq3,dq4};


for k=1:n
    for j=1:n
        c=0;
        for i=1:n
            cr=(Mp{i}(k,j)+Mp{j}(k,i)-Mp{k}(i,j))*qp{i};
            c=c+cr;
        end
        C(k,j)=(1/2)*(c);
    end
end


syms G P g real

% gravitational acceleration
vg=[0;0;-g];

% Potential Energy
P=0;
for i=1:n
    tc{i+1};
    p=(m{i}*vg'*tc{i+1});
    P=P+p;
end
P=(P);
% fprintf('P');

% disp(P)
% Gravitational force vector
q={q1,q2,q3,q4};
for i=1:n
    G(i,1)=(diff(P,q{i}));
end
% fprintf('G');
% disp(G)

end