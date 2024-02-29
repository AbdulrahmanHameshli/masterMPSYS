clear all; close all; clc;

lc = lines(7);
if ~ exist(fullfile(pwd,'images'),'dir'), mkdir images; end

% lt = @(s) clipboard('copy',latex(s));

% B

syms v_a T_e R L K_e K_t D_1 D_2 J_1 J_2 B

ia      = sym('i_a_',[1,2]);
phi1    = sym('phi_1_',[1,2]);
w1      = sym('omega_1_',[1,2]);
phi2    = sym('phi_2_',[1,2]);
w2      = sym('omega_2_',[1,2]);
phi3    = sym('phi_3_',[1,2]);

x = [phi1; w1; phi2; w2; phi3]
u = [v_a; T_e]

eqs = [       
        J_1*w1(2) == K_t*((v_a-K_e*w1(1))/(R)) - D_1*(phi1(1)-phi2(1)) ;
        J_2*w2(2) == D_1*(phi1(1)-phi2(1)) - D_2*(phi2(1)-phi3(1)) ;
        0 == D_2*(phi2(1)-phi3(1)) - B*phi3(2) + T_e;
        phi1(2) == w1(1) ;
        phi2(2) == w2(1) 
    ];
     
eqs = lhs(eqs) - rhs(eqs);

[Am,Bm] = get_state_space(eqs, x(:,2), x(:,1), u)

% A
vars_sym   = [K_e   , K_t   , J_1   , J_2   , B     , D_2]; 
vars_value = [.1    , .1    , 1e-5  , 4e-5  , 2e-3  ,  2];
repl = @(x) subs(x, vars_sym, vars_value);

C1 = [
    0 0 1 0 0      ;
    0 0 0 1 0
]

C2 = [
    0 -K_e/R 0 0 0      ;
    0 0 D_2/B 0 -D_2/B
];

D1 = [
    0 0   ;
    0 0
]

D2 = [
    1/R 0   ;
    0   1/B
]

A = repl(Am)
B = repl(Bm);

C1 = repl(C1);
D1 = repl(D1)

C2 = repl(C2);
D2 = repl(D2);

vars_sym   =  [D_1]; 
vars_value =  [20];
repl = @(x) subs(x, vars_sym, vars_value);
A = repl(A)

det(A)
% Symbolic variable for R   

% Calculate the characteristic polynomial
characteristic_poly = det(A - eye(size(A)) * sym('lambda'))
    

% Solve for the values of R when the real part of any eigenvalue becomes positive
unstable_R_values = solve(characteristic_poly == 0 , R) 

% % Display the critical values of R for instability
% disp('Critical values of R for instability:');
% disp(unstable_R_values)







disp("Not MATLAB functions")
% S matrix
% disp("S matrix rank")
% s = [B A*B A^2*B A^3*B A^4*B]
% rank(s)
% rref(s);
% Controllable

% C matrix case 1
% disp("o_case_1")
% o = [C1; C1*A; C1*A^2; C1*A^3; C1*A^4]
% size(o)
% rank(o)
% rref(o)
% Observable

% C matrix case 2
% disp("o_case_2")
% o = [C2; C2*A; C2*A^2; C2*A^3; C2*A^4]
% size(o)
% rank(o)
% rref(o)
% Not oObservable

% Question B
% Case 1
% Controllable -> stabilizable
% Observable -> detectable

% Case 2
% Controllable -> stabilizable
% Not Observable -> detectable?

% Question C
% S matrix
syms R K_e K_t J_1 J_2 B D_1 D_2;

vars_sym   = [R, K_e, K_t, J_1, J_2, B, D_1, D_2]; 
vars_value = [1, .1, .1, 1e-5, 4e-5, 2e-3, 20, 2];
repl = @(x) double(subs(x, vars_sym, vars_value));

A = repl(Am)
B = repl(Bm);

C1 = repl(C1);
D1 = repl(D1);

C2 = repl(C2);
D2 = repl(D2);

disp("Matlab functions")

S = ctrb(A,B);
disp("S matrix rank")
rank(S)
cond(S)

disp("o_case_1")
o_case_1 = obsv(A,C1);
rank(o_case_1)
cond(o_case_1)

disp("o_case_2")
o_case_2 = obsv(A,C2);
rank(o_case_2)
cond(o_case_2)

% Because of the high values in the condition numbers, the system is very
% senstive to the changes, therefor the system is basically unstable

% Question D
Ts = 0.001;
Ad = expm(A*Ts)

fun = @(t) expm(A*t);

% Question E
syms s t
matrix = inv(eye(size(A))*s-A);
exp_At = vpa(ilaplace(matrix));

Bd = int(exp_At,t,0,1e-3)*B
% Bd = round(Bd, 8)
Bd = double(Bd)










% Question F

disp("eif of Ad ")
eig(Ad)

DT_cont = ctrb(Ad,Bd);
DT_obsv = obsv(Ad,C2);

rref(transpose(DT_cont))
rref(DT_obsv)

rank(DT_cont)
rank(DT_obsv)

%System 1 is controlable and observebility
% The system is stabel if the eid is less then 1. In this system there is
% at least one eigen value withch do not satisfy the criteria. not minimal
% realization becouse 

%system 2 is controlable but not observebility
% The system is stabel if the eid is less then 1. In this system there is
% at least one eigen value withch do not satisfy 
% the criteria. not minimal
% realization becouse it is not observebility 



% Project 3
Ts = 1e-3;

% A 
sigma2_va   = calcVariance(0.3, 0.997);
sigma2_Te   = calcVariance(0.1, 0.997);
sigma2_phi2 = calcVariance(0.02, 0.997);
sigma2_w2   = calcVariance(0.01, 0.997);

R = diag([sigma2_va sigma2_Te sigma2_phi2 sigma2_w2]);  


% B 
Qm = diag([sigma2_va,sigma2_Te]);
Rm = diag([sigma2_phi2 sigma2_w2]);

Q = diag([10 1 1 10 1])
Kf = lqr(A,C1',Q,Rm)'
SYS_kalman= ss(Ad-Kf*C1,[Bd Kf],C1,0)

% disc_sys_case1 = ss(Ad,Bd,C1,0,Ts)
% disc_sys_case2 = ss(Ad,[Bd Bd],C2,0)

% Kalman case 1
% [~, k1, P] = kalman(disc_sys_case1,Qm,Rm,0);
% k1
format long
% disp(P)
% eig(Ad - k1*C1)
% kalman case 2

% does not work becouse of the sysem is unsta;ble 
% [~, k2, P] = kalman(disc_sys_case2,Qm , Rm,[0;0])     ;
% 
% eig(Ad- k2*C1)
% 
% % 
% 
% Q = diag([10 1 0 10 1]);
% 
% L = lqr(disc_sys_case1,Q ,Rm,0);


function [A,b] = get_state_space(eqs, xdot, x, u)
    A = -jacobian(eqs, xdot) \ jacobian(eqs, x);
    b = -jacobian(eqs, xdot) \ jacobian(eqs, u);
end


function sigma2_val = calcVariance(ub, realization_ub)
    syms sigma2 x
    eq = realization_ub == int(1/sqrt(2*pi*sigma2)*exp(-(x^2)/(2*sigma2)),x,-ub,ub);
    sigma2_val = double(solve(eq,sigma2));
    mu = 0;
    pd = makedist('Normal',mu,abs( sqrt(sigma2_val)));
    v_aw = linspace(-ub*2,ub*2,100);
    y = pdf(pd,v_aw);
%     figure('Color','white');
%     plot(v_aw,y)
end
