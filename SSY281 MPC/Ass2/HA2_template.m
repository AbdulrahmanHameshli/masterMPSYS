clear;clc;close all;
% Use this template for Home Assignment 2.
% You may add as many variables as you want but DO NOT CHANGE THE NAME OF
% THE VARIABLES WHICH ARE ALREADY IN THE TEMPLATE.
% Also, DO NOT REUSE THE VARIABLES WHICH ARE ALREADY IN THE TEMPLATE FOR
% SOMETHING ELSE.
% The grading will be partially automatized, so it is very important that
% the names of the variables are what the grader script expects to see.
% All the variables whose name must not be changed have a suffix related to
% the number of the question, so just pay extra attention to them.
% Keep also in mind that the variables are expected to have NUMERICAL
% VALUES and not strings of characters.

%% Q1a

A = [
    1.0041   0.0100   0.0000   0.0000;
     0.8281   1.0041   0.0000  -0.0093;
     0.0002   0.0000   1.0000   0.0098;
     0.0491   0.0002   0.0000   0.9629
     ];

B = [
    0.0007   0.0100;
     0.1398   1.0000;
     0.0028   0.0000;
     0.5605   0.0000
     ];

C = [
    1 0 0 0;
     0 0 1 0
     ];

ys = [pi/18  -pi]';

solution = [
    eye(length(A))-A   -B;
    C       zeros(2)] \[zeros(4,1); ys
          ];


xs_1a = solution(1:4,1)
us_1a = solution(5:end,1)
%% Q1b


B1 = B(:,2);

p = size(C,1);              
m = size(B1,2);             
n = size(A,1);              

Q = eye(p);

H = 2* blkdiag(C'*Q*C, zeros(m));
f = [ 
    (-2*ys'*Q*C)';
    zeros(m,1) 
    ];

Aeq = [eye(n)-A,-B1];
beq = zeros(n,1);

solution = quadprog(H ,f,[],[],Aeq,beq,[],[],[]);

xs_1b = solution(1:4,1)
us_1b = solution(end,1)



%% Q1c


C2 = C(1,:);

p = size(C2,1);              
m = size(B,2);             
n = size(A,1);              
R = eye(m);
constant = pi/18;
H = 2 * blkdiag(zeros(n), R);
f = [ zeros(n,1) ; zeros(m,1) ];
Aeq = [
    eye(n)-A -B; C2 0*C2*B
    ];
beq = [
    zeros(n,1); constant
    ];

solution = quadprog(H ,f,[],[],Aeq,beq,[],[],[],optimoptions('quadprog','Display','iter'));

xs_1c = solution(1:4,1)
us_1c = solution(5:end,1)

%% Q2a
B = B(:,1);
Bp = B;
Cp = [0 1]';

% system 1
Bd1 = zeros(4,1);
Cd1 = [1;1];
Ae_1_2a = [A Bd1; zeros(size(Bd1,2),size(A,2)), eye(size(Bd1,2))];
Be_1_2a = [B; zeros(size(Bd1,2),size(B,2))];
Ce_1_2a = [C Cd1];
Original_sys = rank(obsv(A,C))
augsys1_rank = rank([eye(length(A))-A  -Bd1; C  Cd1 ])

% system 2
Bd2 = zeros(4,2);
Cd2 = eye(2);
Ae_2_2a = [A Bd2; zeros(size(Bd2,2),size(A,2)), eye(size(Bd2,2))];
Be_2_2a = [B; zeros(size(Bd2,2),size(B,2))];
Ce_2_2a = [C Cd2];
augsys2_rank = rank([eye(length(A))-A  -Bd2; C  Cd2 ])

% system 3
Bd3 = [zeros(4,1) Bp];
Cd3 = eye(2);
Ae_3_2a = [A Bd3; zeros(size(Bd3,2),size(A,2)), eye(size(Bd3,2))];
Be_3_2a = [B; zeros(size(Bd3,2),size(B,2))];
Ce_3_2a = [C Cd3];
augsys3_rank = rank([eye(length(A))-A  -Bd3; C  Cd3 ])
%% Q2b

Q = eye(5);
R = eye(2);
[~,Le_1_2b,~] = idare(Ae_1_2a',Ce_1_2a',Q,R)
Q = eye(6);
R = eye(2);
[~,Le_2_2b,~] = idare(Ae_2_2a',Ce_2_2a',Q,R);
[~,Le_3_2b,~] = idare(Ae_3_2a',Ce_3_2a',Q,R)
%% Q2c

H = [0 1];
Mss_1_2c = [
            eye(length(A))-A, -B;
      H*C     zeros(size(H*C,1),size(B,2))]\[Bd1; -H*Cd1]
Mss_2_2c = [
    eye(length(A))-A, -B;
          H*C     zeros(size(H*C,1),size(B,2))]\[Bd2; -H*Cd2];
Mss_3_2c = [
    eye(length(A))-A, -B;
          H*C     zeros(size(H*C,1),size(B,2))]\[Bd3; -H*Cd3]
%% Q2d

% provide plots showing how the states evolve for all the observable systems
clc
close all

p = size(C2,1);               
m = size(B,2);                 
n = size(A,1);                  
N = 50;                         
M = 40;                         
R = 0.1*eye(m);                 
tf = 1000;                      
Pf = diag([5, 2, 0.5, 0.1]);
Q = diag([5, 2, 0.5, 0.1]);     

% model 1
nd = 1;
Mss = Mss_1_2c;
L_est = Le_1_2b';               
A_est = Ae_1_2a;
B_est = Be_1_2a;
C_est = Ce_1_2a;
x = zeros(n,tf);
x0 = [pi/36 0 0 0]';                        
x(:,1)  = x0;
x_est_hat = zeros(n+nd,tf);
x_est_hat(:,1) = [zeros(n,1) ; zeros(nd,1)];
dinit = 50;         
p = 0.2*[zeros(1,dinit), ones(1,tf-dinit)];
ysp = zeros(4,tf); 

% Simulation
    for k = 1:tf
       
        d_hat = x_est_hat(end-nd+1:end, k);     
        xs_us = Mss*d_hat;                      
        xs = xs_us(1:n);                        
        us = xs_us(n+1:end);                    
        x_hat = x_est_hat(1:n, k);
        deviation_x = x_hat - xs;
        u0 = RHC_simulation(A,B,Q,R,Pf,N,M,deviation_x);
        u(:,k) = u0 + us;
        y(:,k) = C*x(:,k) + Cp *  p(k);
        x_est_hat_priori = x_est_hat(:,k) + L_est*(y(:,k) - C_est*x_est_hat(:,k));
        x_est_hat(:,k+1) = A_est*x_est_hat_priori + B_est*u(:,k);
        x(:,k+1) = A*x(:,k) + B*u(:,k) + Bp*p(k);


    end

figure()
subplot(4,1,1)
title('Ball angle \theta_1')
hold on
plot(x(1,1:end-1),'linewidth',2)
plot(ysp(1,1:end-1),'--k')
plot(dinit,x(1,dinit),'xr','MarkerSize',12)
grid on
legend('\theta_1','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad]')

subplot(4,1,2)
title('Ball speed {\partial \theta_1} / {\partial t}')
hold on
plot(x(2,1:end-1),'linewidth',2)
plot(ysp(2,1:end-1),'--k')
plot(dinit,x(2,dinit),'xr','MarkerSize',12)
grid on
legend('{\partial \theta_1} / {\partial t}','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad/s]')

subplot(4,1,3)
title('Wheel angle \theta_2')
hold on
plot(x(3,1:end-1),'linewidth',2)
plot(ysp(3,1:end-1),'--k')
plot(dinit,x(3,dinit),'xr','MarkerSize',12)
grid on
legend('\theta_2','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad]')

subplot(4,1,4)
title('Control u')
hold on
plot(u(1:end-1),'linewidth',2)
plot(dinit,u(1,dinit),'xr','MarkerSize',12)
grid on
legend('input u','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[no unit]')



% Model 3
nd = 2;
Mss = Mss_3_2c;
L_est = Le_3_2b';
A_est = Ae_3_2a;
B_est = Be_3_2a;
C_est = Ce_3_2a;
x = zeros(n,tf);
x0 = [pi/36 0 0 0]';                        
x(:,1)  = x0;
x_est_hat = zeros(n+nd,tf);
x_est_hat(:,1) = [zeros(n,1) ; zeros(nd,1)];

dinit = 50;         
p = 0.2*[zeros(1,dinit), ones(1,tf-dinit)];
ysp = zeros(4,tf); 

    for k = 1:tf
        d_hat = x_est_hat(end-nd+1:end, k);     
        xs_us = Mss*d_hat;                    
        xs = xs_us(1:n);                      
        us = xs_us(n+1:end);                   

        x_hat = x_est_hat(1:n, k);
        deviation_x = x_hat - xs;
        u0 = RHC_simulation(A,B,Q,R,Pf,N,M,deviation_x);
        u(:,k) = u0 + us;
        y(:,k) = C*x(:,k)+ Cp *p(k);
        x_est_hat_priori = x_est_hat(:,k) + L_est*(y(:,k) - C_est*x_est_hat(:,k));
        x_est_hat(:,k+1) = A_est*x_est_hat_priori + B_est*u(:,k);
        x(:,k+1) = A*x(:,k) + B*u(:,k) + Bp*p(k);
    end


figure()
subplot(4,1,1)
title('Ball angle \theta_1')
hold on
plot(x(1,1:end-1),'linewidth',2)
plot(ysp(1,1:end-1),'--k')
plot(dinit,x(1,dinit),'xr','MarkerSize',12)
grid on
legend('\theta_1','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad]')

subplot(4,1,2)
title('Ball speed {\partial \theta_1} / {\partial t}')
hold on
plot(x(2,1:end-1),'linewidth',2)
plot(ysp(2,1:end-1),'--k')
plot(dinit,x(2,dinit),'xr','MarkerSize',12)
grid on
legend('{\partial \theta_1} / {\partial t}','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad/s]')

subplot(4,1,3)
title('Wheel angle \theta_2')
hold on
plot(x(3,1:end-1),'linewidth',2)
plot(ysp(3,1:end-1),'--k')
plot(dinit,x(3,dinit),'xr','MarkerSize',12)
grid on
legend('\theta_2','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad]')

subplot(4,1,4)
title('Control u')
hold on
plot(u(1:end-1),'linewidth',2)
plot(dinit,u(1,dinit),'xr','MarkerSize',12)
grid on
legend('input u','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[no unit]')

function u0 = RHC_simulation(A,B,Q,R,Pf,N,M,deviation_x)
    n=length(deviation_x);           
    m=length(B(1,:));                
    H_pred_horiz = kron(eye(N-1),Q);                 
    H_pred_final = Pf;                                    
    H_contr_horiz = kron(eye(M),R);                   
    H = blkdiag(H_pred_horiz, H_pred_final, H_contr_horiz);
    f = [];
    
    Aeq_states = [zeros(n,n*N);kron(eye(N-1),A),zeros(n*(N-1),n)]-eye(n*N);
    Aeq_control = kron(eye(M),B);  
    Aeq_ub= repmat(Aeq_control((M-1)*n+1:M*n,:),N-M,1);
    Aeq_control = [Aeq_control;Aeq_ub];
    
    Aeq   = [Aeq_states, Aeq_control];

    beq = [-A*deviation_x;zeros(n*(N-1),1)];

    Ain = [];
    Bin = [];

    lb = [];
    ub = [];
    
    sol = quadprog(2*H,f,Ain,Bin,Aeq,beq,lb,ub); 
    u0=sol(n*N+1:n*N+m);
end



