clear all
syms q1 q2 L1 L2 DH real

joints={'R','R'}

%% DH Table
% Assuming the CM are located at the middle of the links

DH=[q1      0     L1   0;
    q2      0     L2   0;
    q1      0     L1/2 0;
    q2      0     L2/2 0]

%% Relative Homogeneous Transformations
% DOFs
n=size(DH,1)/2

% H: Homogeneous Transformations links
% Hc: Homogeneous Transformations cm

for i=1:n
    i
    H{i}=dhHT(DH(i,:));
    disp(H{i})
    Hc{i}=dhHT(DH(i+n,:));
    disp(Hc{i})
end

%Verify the Hs

%% Absolute HT Link

%Links
% H1_0 is already wrt 0
aH{1}=H{1};

disp(aH{1})

% H2_0=H1_0*H2_1
% H3_0=H2_0*H3_2
%...
for i=2:n
    aH{i}=simplify(aH{i-1}*H{i});

    disp(aH{i})
end

%% absolute HT CM
%CM
% Hcm1_0 is already wrt 0
aHc{1}=Hc{1};
disp(aHc{1})

% Hcm2_0=H1_0*Hcm2_1
% Hcm3_0=H2_0*Hcm3_2
%...
for i=2:n
    aHc{i}=simplify(aH{i-1}*Hc{i});

    disp(aHc{i})
end

%Verify the Hs

%% Axes of motion and position vectors
syms z3d real

% Z0_0=[0;0;1]
z{1}=[0;0;1]
% P0_0=[0;0;0];
t{1}=[0;0;0]



for i=1:n
    i
    % axis of motion third 3D column vector absolute H
    z{i+1}=aH{i}(1:3,3);
    % link position vector fourth 3D column vector absolute H
    t{i+1}=aH{i}(1:3,4);

    % cm position vector fourth 3D column vector absolute H
    tc{i+1}=aHc{i}(1:3,4);

    % Rotiation Matrix CM
    Rc{i}=aHc{i}(1:3,1:3);

    disp(z{i+1})
    disp(t{i+1})
    disp(tc{i+1})
    disp(Rc{i})

end




%% Jacobian ef
% syms Jef_0 real
% 
% Jef_0 = Jac(t,z,t{end},n,joints);
% disp(Jef_0)
% 
% % Validation
% 
% Jval=simplify([diff(t{end},q1), diff(t{end},q2)]);
% 
% DJ=simplify(Jef_0(1:3,:)-Jval)

%% Jacobians CM
syms Jcm_0 J real

Jcm_0={'','',''};

for j=1:n
    j
    Jcm_0{j}=Jac(t,z,tc{j+1},j,joints);
    disp(Jcm_0{j})

    % Validation
    Jval=simplify([diff(tc{j+1},q1), diff(tc{j+1},q2)]);
    DJcm=simplify(Jcm_0{j}(1:3,:)-Jval)


end


%% Mass
syms m1 m2 M real
syms I111 I112 I113 I122 I123 I1 real
syms I211 I212 I213 I222 I223 I2 real

% Inertia Tensor
I{1}=[I111 I112 I113;I112 I122 I123;I113 I123 I1]
I{2}=[I211 I212 I213;I212 I222 I223;I213 I223 I2]

m={m1;m2}

M=0;

for i=1:n
    Jv=Jcm_0{i}(1:3,:);
    Jw=Jcm_0{i}(4:6,:);
    Mt=simplify(m{i}*Jv'*Jv+Jw'*Rc{i}*I{i}*Rc{i}'*Jw);
    disp(Mt)
    M=simplify(M+Mt);
end

M=simplify(M);

disp(M)

% Validation

DM=simplify(M-M')

%% C Matrix
syms q1p q2p C real

Mp{1}=simplify(diff(M,q1));
disp(Mp{1})
Mp{2}=simplify(diff(M,q2));
disp(Mp{2})


qp={q1p,q2p}


for k=1:n
    for j=1:n
        c=0;
        for i=1:n
            cr=simplify((Mp{i}(k,j)+Mp{j}(k,i)-Mp{k}(i,j))*qp{i});
            c=c+cr;
        end
        C(k,j)=(1/2)*simplify(c);
    end
end

disp(C)

%Validation 


dM=Mp{1}*q1p+Mp{2}*q2p;

N=simplify(dM-2*C)

syms x1 x2 real

x=[x1;x2];

DN=simplify(x'*N*x)


%% G
syms G P g real

% gravitational acceleration
vg=[g;0;0]

% Potential Energy
P=0;
for i=1:n
    tc{i+1}
    p=simplify(m{i}*vg'*tc{i+1});
    P=P+p;
end
P=simplify(P);

% Gravitational force vector
q={q1,q2}
for i=1:n
    G(i,1)=simplify(diff(P,q{i}));
end

disp(G)

%% Dynamic Model
syms q1ppr q2ppr q1pr q2pr real

qpr=[q1pr;q2pr]
qppr=[q1ppr;q2ppr]


M
C
G

% Equations of motion
tau=expand(M*qppr+C*qpr+G)

%% Regressor

Theta=[I1;
       I2;
       L1^2*m1;
       L1^2*m2;
       L2^2*m2;
       L1*g*m1;
       L1*g*m2;
       L2*g*m2;
       L1*L2*m2];

size(Theta)

Yr=[q1ppr, q1ppr+q2ppr, q1ppr/4, q1ppr, (q1ppr+q2ppr)/4, -sin(q1)/2, -sin(q1), -(cos(q1)*sin(q2))/2-(cos(q2)*sin(q1))/2, q1ppr*cos(q2) + (q2ppr*cos(q2))/2 - (q1p*q2pr*sin(q2))/2 - (q2p*q1pr*sin(q2))/2 - (q2p*q2pr*sin(q2))/2;
        0, q1ppr+q2ppr,       0,     0, (q1ppr+q2ppr)/4,         0,        0, -(cos(q1)*sin(q2))/2-(cos(q2)*sin(q1))/2, (q1ppr*cos(q2))/2 + (q1p*q1pr*sin(q2))/2]

size(Yr)

% -(cos(q1)*sin(q2))/2 -(cos(q2)*sin(q1))/2 = -sin(q1 + q2)/2


% Validation
Dt=simplify(tau-Yr*Theta)


