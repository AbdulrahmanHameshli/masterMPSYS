clear;clc;close all;
% Use this template for Home Assignment 1.
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

%% Q1a (discretization of a state-space model)

% Discrete Model
a = -0.9421;
b = 82.7231;
c = 14.2306;
p = -3.7808;
q = 4.9952; 
r = 57.1120;
h = 0.1;
tau = 0.8 * h;




A_contin   = [0 1 0 0;
        b 0 0 a; 
        0 0 0 1; 
        q 0 0 p];

B_contin = [  0; 
        c; 
        0; 
        r];

dsys = expm([A_contin, B_contin; [0 0 0 0], 0].*h);


A_1a =  dsys(1:4,1:4)
B_1a =  dsys(1:4,5)    
C_1a =  [1 0 0 0]
eig_1a = eig(A_1a)

%% Q1b
% Delayed model

csys = [A_contin B_contin; zeros(1,5)];
dsys_tau = expm([A_contin, B_contin; [0 0 0 0], 0].*tau);
dsys = expm(csys*h);
[row_a, col_a] = size(A_contin);
[row_b, col_b] = size(B_contin);
B_tau = dsys_tau(1:row_b,col_a+1:end);
dsys_delayed = expm(csys*(h-tau));
A_h_tau = dsys_delayed(1:row_a,1:row_a);
B_h_tau = dsys_delayed(1:row_b,row_a+1:end);
B1 = A_h_tau * B_tau;
B2 = B_h_tau;
A = dsys(1:row_a,1:col_a);


Aa_1b = [A B1; zeros(1,5)]
Ba_1b = [B2; 1]
Ca_1b = [C_1a 0]
eig_1b = eig(Aa_1b)

%% Q2a (Dynamic Programming solution of the LQ problem)

% finite horizon problem
clc;clear;
A = [1.0041 0.0100 0 0;
    0.8281 1.0041 0 -0.0093; 
    0.0002 0.0000 1 0.0098;
    0.0491 0.0002 0 0.9629];

B = [0.0007; 
    0.1398;
    0.0028;
    0.5605];

Q = eye(4);

Pf = 10*eye(4);

R = 1;


[eig_v ,N_2a, K_2a, ~] = MinNQ2a(A,B,Q,R,Pf)




%% Q2b

% infinite horizon problem
clc; clear; 
A = [1.0041 0.0100 0 0; 0.8281 1.0041 0 -0.0093; 0.0002 0.0000 1 0.0098; 0.0491 0.0002 0 0.9629];
B= [0.0007; 0.1398; 0.0028; 0.5605];
Q = eye(4);
Pf = 10*eye(4);
R = 1;


[P_inf_2b] = idare(A,B,Q,R,[],[])
[numIterations, P_inf_DP_2b] = Q2b(A, B, Q, Pf, R)


%% Q2c
clc; 
A = [1.0041 0.0100 0 0; 0.8281 1.0041 0 -0.0093; 0.0002 0.0000 1 0.0098; 0.0491 0.0002 0 0.9629];
B  = [0.0007; 0.1398; 0.0028; 0.5605];
Q = eye(4);
Pf = P_inf_DP_2b;
R = 1;

[~,~,K_2C,N_2C] = MinNQ2a(A,B,Q,R,Pf)

%% Q3a (batch solution of the LQ problem)
MaxNumber = 100;
firstNumber = 1;

A= [
    1.0041 0.0100 0 0;
    0.8281 1.0041 0 -0.0093;
    0.0002 0.0000 1 0.0098;
    0.0491 0.0002 0 0.9629    
];
B= [
    0.0007;
    0.1398;
    0.0028;
    0.560
];
Q= eye(4);
R= 1;
x_0= [1;1;1;1];
N= 32;
P_f= 10*eye(4);
for i = 1:100
    N = i;
    omega_list = [];
    for i=1:N
        omega_list = [omega_list; A^i];
    end
       gamma_list = [];
    nollor_array = zeros(length(B),1);
    lr = [];
   
    for i=1:N
        lr = [lr A^(N-i)*B];
    end  
    gamma_list = [lr];
    for i=1:N-1
        ar = [lr(:,i+1:end)];
    
        for j=1:i
            ar = [ar, nollor_array];
        end
        
        gamma_list = [ar; gamma_list];
    end
    bar_q = [];
    for i=1:N-1
        bar_q = blkdiag(bar_q,Q);
    end
    bar_q = blkdiag(bar_q,P_f);
    bar_r = [];
    for i=1:N
        bar_r = blkdiag(bar_r,R);
    end
    kb = -inv(gamma_list'*bar_q*gamma_list+bar_r)*gamma_list'*bar_q*omega_list;
    u_star = kb * x_0;
    poles = abs(eig(A+(B*kb(1,:))));
    state = 0;
    for i=1:length(poles)
    
        if poles(i) >= 1
            state = state + 1;
        end
    end
    if (state > 0)
    else
        N_3a = N
        K0_3a = kb(1,:)
        break
    end
end
%% Q4 (Receding horizon control)

% plot x1, x2, x3 and u as per the instructions
clc; clear; close all; 

% case1
r = 1;
numIterations= 38;
[x_ploting1, u_ploting1] = Q4(numIterations, r);

% case2
r = 1;
numIterations= 78;
[x_ploting2, u_ploting2] = Q4(numIterations, r);
% case3
r = 0.1;
numIterations= 38;
[x_ploting3, u_ploting3] = Q4(numIterations, r);
% case4
r = 0.1;
numIterations= 78;
[x_ploting4, u_ploting4] = Q4(numIterations, r);



x1 = linspace(0,2000,2000);
x2 = linspace(0,100,100);
x3 = linspace(0,30,30);

% Initialize arrays to store data
y1 = zeros(4, 100);
y2 = zeros(4, 100);
y3 = zeros(4, length(x1));
u = zeros(4, length(x3));

% Loop through cases
for i = 1:4
    y1(i,:) = eval(sprintf('x_ploting%d(1,1:100)', i));
    y2(i,:) = eval(sprintf('x_ploting%d(2,1:100)', i));
    y3(i,:) = eval(sprintf('x_ploting%d(3,1:end-1)', i));
    u(i,:) = eval(sprintf('u_ploting%d(1,1:30)', i));
end

figure;

subplot(4,1,1);
plot(x2, y1, 'LineWidth', 2);
title('Angle of ball (theta 1)');
legend('show');
legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
xlabel('Data sample');
ylabel('Angle [rad]');

subplot(4,1,2);
plot(x2, y2, 'LineWidth', 2);
title('Angle velocity of ball (theta 1 dot)');
legend('show');
legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
xlabel('Data sample');
ylabel('Angle velocity [Rad/s]');

subplot(4,1,3);
plot(x1, y3, 'LineWidth', 2);
title('Angle of wheel (theta 2)');
legend('show');
legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
xlabel('Data sample');
ylabel('Angle [rad]');

subplot(4,1,4);
plot(x3, u, 'LineWidth', 2);
title('Input torque');
legend('show');
legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
xlabel('Data sample');
ylabel('Torque [Nm]');

%% Q5 (constrained receding horizon control)
 

clc;clear;close all; 
A = [1.0041 0.0100 0 0; 0.8281 1.0041 0 -0.0093; 0.0002 0.0000 1 0.0098; 0.0491 0.0002 0 0.9629];
B = [0.0007; 0.1398; 0.0028; 0.5605];
C = [1 0 0 0; 0 0 1 0];
Q = eye(4);
Pf = 10*eye(4);
R = 1;
x0 = [pi/38; 0; 0; 0]; 
N = 40;
no_of_states = 4;
x2_max = 1;
u_max = 8; 

[x_values_1, u_values_1] = Q5(A, B, Q, Pf, R, x0, N, no_of_states, x2_max, u_max);
disp("25% completed")
R = 1;
N = 80;
[x_values_2, u_values_2] = Q5(A, B, Q, Pf, R, x0, N, no_of_states, x2_max, u_max);
disp("50% completed")
R = 0.1;
N = 40;
[x_values_3, u_values_3] = Q5(A, B, Q, Pf, R, x0, N, no_of_states, x2_max, u_max);
disp("75% completed")
R = 0.1;
N = 80;
[x_values_4, u_values_4] = Q5(A, B, Q, Pf, R, x0, N, no_of_states, x2_max, u_max);
disp("100% completed")

% [m,n] = size(x_ploting);
numIterations = 2000;
x1 = linspace(0, numIterations, numIterations);
x2 = linspace(0, numIterations, numIterations);
x3 = linspace(0, 100, 100);
x4 = linspace(0, 30, 30);

x_values = {x_values_1, x_values_2, x_values_3, x_values_4};
u_values = {u_values_1, u_values_2, u_values_3, u_values_4};

figure;

for i = 1:4
    y1 = x_values{i}(1, 1:100);
    y2 = x_values{i}(2, 1:100);
    y3 = x_values{i}(3, :);
    u = u_values{i}(1:30);

    subplot(4,1,1);
    plot(x3, y1, 'LineWidth', 2)
    hold on;

    subplot(4,1,2);
    plot(x3, y2, 'LineWidth', 2)
    hold on;

    subplot(4,1,3);
    if i == 1 || i == 3
        plot(x1, y3, 'LineWidth', 2)
    else
        plot(x2, y3, 'LineWidth', 2)
    end
    hold on;

    subplot(4,1,4);
    plot(x4, u, 'LineWidth', 2)
    hold on;
end

subplot(4,1,1);
title('Angle of ball (theta 1)');
legend('show');
legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
xlabel('Data sample');
ylabel('Angle [rad]');

subplot(4,1,2);
title('Angle velocity of ball (theta 1 dot)');
legend('show');
legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
xlabel('Data sample');
ylabel('Angle velocity [rad/s]');

subplot(4,1,3);
title('Angle of wheel (theta 2)');
legend('show');
legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
xlabel('Data sample');
ylabel('Angle [rad]');

subplot(4,1,4);
title('Input torque');
legend('show');
legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
xlabel('Data sample');
ylabel('Torque [Nm]');

% x1 = linspace(0,2000,numIterations);
% x2 = linspace(0,2000,numIterations);
% x3 = linspace(0,100,100);
% x4 = linspace(0,30,30);
% y1_1 = x_values_1(1,1:100);
% y2_1 = x_values_2(1,1:100);
% y3_1 = x_values_3(1,1:100);
% y4_1 = x_values_4(1,1:100);
% y1_2 = x_values_1(2,1:100);
% y2_2 = x_values_2(2,1:100);
% y3_2 = x_values_3(2,1:100);
% y4_2 = x_values_4(2,1:100);
% y1_3 = x_values_1(3,:);
% y2_3 = x_values_2(3,:);
% y3_3 = x_values_3(3,:);
% y4_3 = x_values_4(3,:);
% u_1 = u_values_1(1:30);
% u_2 = u_values_2(1:30);
% u_3 = u_values_3(1:30);
% u_4 = u_values_4(1:30);
% 
% figure;
% 
% subplot(4,1,1);
% plot(x3,y1_1,'LineWidth', 2)
% hold on;
% plot(x3,y2_1,'LineWidth', 2)
% hold on;
% plot(x3,y3_1,'LineWidth', 2)
% hold on;
% plot(x3,y4_1,'LineWidth', 2)
% hold on;
% legend('show');
% legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
% title('Angle of ball (theta 1)');
% xlabel('Data sample');
% ylabel('Angle [rad]');
% 
% subplot(4,1,2);
% plot(x3,y1_2,'LineWidth', 2)
% hold on;
% plot(x3,y2_2,'LineWidth', 2)
% hold on;
% plot(x3,y3_2,'LineWidth', 2)
% hold on;
% plot(x3,y4_2,'LineWidth', 2)
% hold on;
% legend('show');
% legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
% title('Angle velocity of ball (theta 1 dot)');
% xlabel('Data sample');
% ylabel('Angle velocity [rad/s]');
% 
% subplot(4,1,3);
% plot(x1,y1_3,'LineWidth', 2)
% hold on;
% plot(x2,y2_3,'LineWidth', 2)
% hold on;
% plot(x1,y3_3,'LineWidth', 2)
% hold on;
% plot(x2,y4_3,'LineWidth', 2)
% hold on;
% legend('show');
% legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
% title('Angle of wheel (theta 2)');
% xlabel('Data sample');
% ylabel('Angle [rad]');
% 
% subplot(4,1,4);
% plot(x4,u_1,'LineWidth', 2)
% hold on;
% plot(x4,u_2,'LineWidth', 2)
% hold on;
% plot(x4,u_3,'LineWidth', 2)
% hold on;
% plot(x4,u_4,'LineWidth', 2)
% hold on;
% legend('show');
% legend(["R=1 & N = 40", "R=1 & N = 80", "R=0.1 & N = 40", "R=0.1 & N = 80"])
% title('Input torque');
% xlabel('Data sample');
% ylabel('Torque [Nm]');















function [eig_v, N_out_2A , K_out,N_out_2B] = MinNQ2a(A_in, B_in, Q_in, R_in, Pf_in)
P = Pf_in;
N_out_2B = 1;
N_out_2A = 1;


while N_out_2A <= 1000
N_out_2A = N_out_2A + 1;
    P = Q_in + A_in' * P * A_in - A_in' * P * B_in * inv(R_in + B_in' * P * B_in) * B_in' * P * A_in;
    K_out = -inv(R_in + B_in' * P * B_in) * B_in' * P * A_in;
    poles = abs(eig(A_in + B_in * K_out));
    eig_v = poles;
    unstbl = sum(poles >= 1);
    if unstbl == 0
        break
    end
N_out_2B = N_out_2B +1;
end

end


function [numIterations, currentP] = Q2b(A, B, Q, Pf, R)
    currentP = Q + A' * Pf * A - A' * Pf * B * inv((R + B' * Pf * B)) * B' * Pf * A;
    numIterations = 2;
    while true
        numIterations = numIterations + 1;
        nextP = Q + A' * currentP * A - A' * currentP * B * inv((R + B' * currentP * B)) * B' * currentP * A;
        if norm(nextP - currentP) <= 1e-1
            break;
        end
        currentP = nextP;
    end
end



function [x_plotting, u_plotting] = Q4(numIterations, r)
    a = [1.0041 0.0100 0 0; 0.8281 1.0041 0 -0.0093; 0.0002 0.0000 1 0.0098; 0.0491 0.0002 0 0.9629];
    b = [0.0007; 0.1398; 0.0028; 0.5605];
    c = [1 0 0 0; 0 0 1 0];
    q = eye(4);
    pf = 10*eye(4);
    x0 = [pi/38; 0; 0; 0];
    numTimesteps = 2000;
    x_plotting = x0;
    u_plotting = [];
    for j = 1:numTimesteps
        K_list = [];
        K1 = -inv((r+b'*pf*b))*b'*pf*a;
        K_list = [K_list; K1];
        p1 = q + a'*pf*a + a'*pf*b*K1;
        p_list = p1;
        
        for i = 1:numIterations
            K_next = -inv((r+b'*p_list*b))*b'*p_list*a;
            K_list = [K_next; K_list];
            p_next = q + a'*p_list*a + a'*p_list*b*K_next;
            p_list = p_next;
        end
        
        x_list = [];
        x_list_cal = [];
        x_prev = x0;
        for i = 1:size(K_list,1)
            x_next = a*x_prev + b*K_list(i,:)*x_prev;
            x_list = [x_list; x_prev];
            x_list_cal = [x_list_cal x_prev];
            x_prev = x_next;
        end
        x_list = [x_list; x_prev];
        x_list_cal = [x_list_cal x_prev];
        
        u_list = [];
        for k = 1:size(K_list,1)
            u_list = [u_list K_list(k,:)*x_list_cal(:,k)];
        end
        x0 = x_list(5:8);
        x_plotting = [x_plotting x0];
        u_plotting = [u_plotting u_list(:,1)];
    end
end




function [x_values, u_values] = Q5(A, B, Q, Pf, R, x0, N, no_of_states, x2_max, u_max)
    % Initialize variables
    x_values = [];
    u_values = [];

    for j = 1:2000
        % Objective function
        qbar = blkdiag(kron(eye(N-1), Q), Pf);
        rbar = kron(eye(N), R);
        H    = 2*blkdiag(qbar, rbar);
        f    = [];

        % Equality constraints
        I    = eye(no_of_states);
        Aeq1 = kron(eye(N), I) + kron(diag(ones(N-1, 1), -1), -A);
        Aeq2 = kron(eye(N), -B);
        Aeq  = [Aeq1, Aeq2];
        beq  = [A*x0; zeros(no_of_states*(N-1), 1)];

        % Inequality constraints
        F      = kron([eye(N); zeros(N)], [0 1 0 0; 0 -1 0 0]);
        G      = kron([zeros(N); eye(N)], [1; -1]);
        b_in   = [x2_max*ones(2*N, 1); u_max*ones(2*N, 1)];

        Ain    = [F, G];
        bin    = [x2_max*ones(2*N, 1); u_max*ones(2*N, 1)];

        [Z, ~, ~, ~, ~] = quadprog(H, f, Ain, bin, Aeq, beq);

        x      = Z(1:no_of_states*N);
        u      = Z(no_of_states*N+1:end);
        u0     = u(1);
        x = reshape(x, [4, N]);

        x_values = [x_values, x0];
        u_values = [u_values, u0];
        x0 = x(:, 1);
    end
end

