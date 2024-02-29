clc
close all
clear all
%% Q3c
% solve the LP using the formulation shown in (5)

A = [0.4889   0.2939;
     1.0347  -0.7873;
     0.7269   0.8884;
     -0.3034  -1.1471];
B = [-1.0689;
     -0.8095;
     -2.9443;
      1.4384];
[rows_A, cols_A] = size(A);
objective_function = [zeros(cols_A, 1); 1];  
constraint_matrix = [A, -ones(rows_A, 1);
                     -A, -ones(rows_A, 1)];
constraint_vector = [B; -B];
[optimization_result, min_epsilon, ~, ~, lagrange_multipliers] = linprog(objective_function, constraint_matrix, constraint_vector);
inequality_multipliers = lagrange_multipliers.ineqlin;

z_3c = optimization_result

%% Q3e
% solve the dual problem derived in Q3d
[m, n] = size(A);
f = [B; -B];  
Aeq = [A, -ones(m, 1);
       -A, -ones(m, 1)];
beq = -[zeros(n, 1); 1];
lb = zeros(length(f), 1);
ub = [];
[solution, fval, ~, ~, lambda] = linprog(f, [], [], Aeq', beq, lb, ub);

mu_3e = solution

%% Q3f
% Solve the primal problem by using the solution of the dual problem
% obtained in Q3e

% Define constraint matrix and vector
C = [A, -ones(m, 1); -A, -ones(m, 1)];
b_vec = [B; -B];

% Define objective function coefficients
coeffs_obj = [zeros(n, 1)', 1];

% Identify active constraints
active_idx = (mu_3e ~= 0);
active_C = C(active_idx, :);
active_b = b_vec(active_idx, :);

% Solve the linear problem
z_3f = active_C \ active_b

%% Q4a
% solve the QP
A = 0.4;
B =1;
n = 4;
x0 = 1.5;
H = eye(n);
f = zeros(n, 1);
Aeq = [1, 0, -B, 0;
       -A, 1, 0, -B];
beq = [A * x0; 0];
Aineq = [-1, 0, 0, 0;
          1, 0, 0, 0;
          0, -1, 0, 0;
          0, 1, 0, 0;
          0, 0, -1, 0;
          0, 0, 1, 0;
          0, 0, 0, -1;
          0, 0, 0, 1];
bineq = [-2.5; 5; 0.5; 0.5; 2; 2; 2; 2];
[sol, fval, ~, ~, lambda] = quadprog(H, f, Aineq, bineq, Aeq, beq);
x = sol(1:2);
u = sol(3:4);
Aineq' * lambda.ineqlin;
Aeq' * lambda.eqlin;
mu = lambda.ineqlin
lambda = lambda.eqlin

x_4a = x
u_4a = u

X_star =[x_4a;u_4a]
