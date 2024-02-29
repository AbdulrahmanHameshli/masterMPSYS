

clc
close all
clear
%% Q1a
% represent the polyhedron in V- and H-representation and plot the two
% versions separately

% Define H-representation
A = [0  1;
    -1  0;
    -1 -1;
     1  1];
b = [0 0 1 1]';
H_representatin = Polyhedron(A, b);

V = [0 0; 1 0; 0 -1]; 

R = [1 -1];
v_representatin = Polyhedron('V', V, 'R', R);

figure()
subplot(1,2,1)
plot(H_representatin)
title('H-representation')
subplot(1,2,2)
plot(v_representatin)
title('V-representation')

P_H_1a = H_representatin;
P_V_1a = v_representatin;
%% Q1b
% perform some sets operations

% Close all figures and clear workspace
close all
clear all

% Define Polyhedron P
A1 = [0  1;
      1  0;
      0 -1;
     -1  0];
b1 = [2 2 2 2]';
P = Polyhedron('A', A1, 'b', b1);

% Define Polyhedron Q
A2 = [-1 -1;
       1  1;
       1 -1;
      -1  1];
b2 = [1 1 1 1]';
Q = Polyhedron('A', A2, 'b', b2);

% Define different combinations of polyhedrons
p1 = P;
p2 = Q;
p3 = P + Q;
p4 = P - Q;
p5 = (P - Q) + Q;
p6 = (P + Q) - Q;
p7 = (Q - P) + P;
p8 = (Q + P) - P;

% Plot the polyhedrons
figure()
titles = {'P', 'Q', 'P+Q', 'P-Q', '(P-Q)+Q', '(P+Q)-Q', '(Q-P)+P', '(Q+P)-P'};
for i = 1:8
    subplot(4, 2, i)
    plot(eval(sprintf('p%d', i)))
    title(titles{i})
    axis([-3 3 -3 3])
end





P_1b = p1;
Q_1b = p2;
Mink_sum_1b =p3;
Pontr_diff_1b = p4;
PminusQplusQ_1b = p5;
PplusQminusQ_1b =p6;
QminusPplusP_1b = p7;
QplusPminusP_1b =p8;
%% Q2a
% show that the set S is positively invariant for the system and plot it

% ROBOT ARM MOVEMENT SIMULATION
close all
clear all
clc
A = [0.8   0.4;
    -0.4   0.8];
Ain = [1  0;
     0  1;
    -1  0;
     0 -1;
     1  1;
     1 -1;
    -1  1;
    -1 -1];
bin = [1 1 1 1 1.5 1.5 1.5 1.5]';
S = Polyhedron('A', Ain, 'b', bin);
Reachable = Polyhedron('A', Ain * inv(A), 'b', bin);
plot(S,'color',[0/255 0/255 100/255])
hold on
plot(Reachable,'Colors',[0/255 0/255 60/255])
legend('Initial Position', 'Reachable Positions')


S_2a = S;
%% Q2b
% calculate the one-step reachable set from S and plot it together with S
% Clear the workspace
close all
clc


B = [0 1]';

Au = [1 -1]';
bu = [1 1]';

% polyhedron S is defined in previous question

% Define set U
U = Polyhedron('lb',-1,'ub',1);

% Reachable set
AA = [Ain*inv(A)   -Ain*inv(A)*B
      zeros(2,2)         Au];

bb = [bin; bu];


reach_S = Polyhedron('A',AA,'b',bb);

% Projection
%S_Reach_proj = A*S + B*U;
reash_s_pr = projection(reach_S,[1:2]);



% Plot
figure()
subplot(1,2,1)
plot(reach_S,'color',[0/255 100/255 60/255])
hold on
plot(S,'color',[0/255 0/255 255/255])
title('S and S_{tot} in 3D')
legend('S_{tot}','S')
xlabel('x_1','FontSize',12)
ylabel('x_2','FontSize',12)
zlabel('u','FontSize',12)

subplot(1,2,2)
plot(reash_s_pr,'color',[0/255 80/255 60/255],'alpha',0.5)
hold on
plot(S,'color',[0/255 0/255 60/255])
title('S and reach(S). A projection on x_1-x_2 plane')
legend('reach(S)','S')
xlabel('x_1','FontSize',12)
ylabel('x_2','FontSize',12)


reach_set_S_2b = reash_s_pr;
%% Q2c
% calculate the one-step precursor set of S and plot it together with S
close all
clc

U = Polyhedron('lb', -1, 'ub', 1);
A_aug = [Ain * A, Ain * B; zeros(2), Au];
b_aug = [bin; bu];
S_pre = Polyhedron('A', A_aug, 'b', b_aug);
S_pre_proj = projection(S_pre, [1, 2]);


figure();
subplot(1, 2, 1);
plot(S_pre, 'color', [0/255 100/255 60/255], 'alpha', 0.5);
hold on;
plot(S, 'color', [0/255 0/255 60/255]);
title('S and S_{tot} in 3D');
legend('S_{tot}', 'S');
xlabel('x_1', 'FontSize', 12);
ylabel('x_2', 'FontSize', 12);
zlabel('u', 'FontSize', 12);

subplot(1, 2, 2);
plot(S_pre_proj, 'color', [0/255 100/255 60/255], 'alpha', 0.5);
hold on;
plot(S, 'color', [0/255 0/255 60/255]/ 255);
title('Projection of Pre(S) onto x_1-x_2 Plane');
legend('pre(S)', 'S');
xlabel('x_1', 'FontSize', 12);
ylabel('x_2', 'FontSize', 12);

pre_set_S_2c = 'One-step precursor set of S'; % project tau into 1st and 2nd dimensions
%% Q3a
% find shortest prediction horizon such that the RH controller is feasible
% Clear workspace, close figures, and clear command window

% Clear workspace, close figures, and clear command window
close all
clear all
clc

A = [0.9, 0.4; -0.4, 0.9];
B = [0; 1];
x0 = [2; 0];

model = LTISystem('A', A, 'B', B);

model.u.min = -0.1;
model.u.max = 0.1;
model.x.min = [-3; -3];
model.x.max = [3; 3];

Q = eye(2); 
R = 1;     

model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);

Xf = Polyhedron([0, 0]);
model.x.with('terminalSet');
model.x.terminalSet = Xf;

min_horizon = 20;  
max_horizon = 50;  

for horizon = min_horizon:max_horizon
    mpc_controller = MPCController(model, horizon);
    [~, feasible, open_loop_data] = mpc_controller.evaluate(x0);
    if feasible
        fprintf('Feasible solution found at horizon N = %d\n', horizon)
        break
    end
end

figure('Position', [150, 250, 500, 500]);
subplot(2, 1, 1);
hold on;
plot(0:horizon, open_loop_data.X(1, :), 'linewidth', 2, 'Color', [0/255 100/255 60/255]);
plot(0:horizon, open_loop_data.X(2, :), 'linewidth', 2, 'Color', [0/255 100/255 60/255] * 2);
plot(0:horizon-1, open_loop_data.U, 'linewidth', 2);
legend('x_1', 'x_2', 'u');
xlabel('k'); ylabel('x,u');
title('System states and input', 'FontSize', 16);
grid on;

subplot(2, 1, 2);
plot(open_loop_data.X(1, :), open_loop_data.X(2, :), 'linewidth', 2, 'color', [0/255 100/255 60/255]);
hold on;
plot(0, 0, 'o', 'MarkerFaceColor', [0/255 100/255 60/255], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
plot(x0(1), x0(2), 'o', 'MarkerFaceColor', 'auto', 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
legend('state trajectory', 'terminal set', 'initial set');
xlabel('x_1'); ylabel('x_2');
title('System trajectory', 'FontSize', 16);
xlim([-1.7, 2.1]); ylim([-2, 1.5]);
grid on;


N_3a = horizon;
%% Q3b
% find out if the RH controller is persistently feasible all the way to the
% origin

% Clear command window and close all figures
clc
close all

N = 2;
Cinf = model.invariantSet();
model.x.with('terminalSet');
model.x.terminalSet = Cinf;
mpc_controller = MPCController(model, N);
[~, feasible, ~] = mpc_controller.evaluate(x0);

if feasible
    fprintf('Controller is still feasible!\n')
else
    fprintf('Controller infeasible...\n')
end
X = Polyhedron('lb',model.x.min,'ub',model.x.max);


% Plot
figure();
plot(Cinf, 'color', [0/255 100/255 60/255], 'alpha', 0.5);
hold on;
plot(0, 0, 'o', 'MarkerFaceColor', [0/255 100/255 60/255], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
plot(2, 0, 'o', 'MarkerFaceColor', 'auto', 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
plot(open_loop_data.X(1, :), open_loop_data.X(2, :), 'linewidth', 2, 'color', [0/255 100/255 60/255]);
legend('Xf = C_{\infty}', 'Terminal Set', 'Initial Set', 'Trajectory');


maxContrInvSet_3b = 'Maximal control invariant set for the system';
%% Q3c
% plot the set of feasible initial states for the two controllers
% previously designed and discuss about the size of the problems
% Close all figures and clear command window
% Initialize

% Clear previous figures and command window
% Close all plots and clear command window
close all
clc
U = Polyhedron('lb',-1,'ub',1);

% Define the model with constraints
sys = LTISystem('A', A, 'B', B);
sys.x.min = [-3; -3];
sys.x.max = [3; 3];
sys.u.min = -0.1;
sys.u.max = 0.1;

% Define terminal state constraint sets
terminal_cont1 = Polyhedron([0 0]);         % Just the origin
terminal_cont2 = sys.invariantSet();        % Entire control-invariant set

% Calculate feasible initial states for each controller
initial_cont1 = sys.reachableSet('X', terminal_cont1, 'U', U, 'N', 26, 'direction', 'backward');
initial_cont2 = sys.reachableSet('X', terminal_cont2, 'U', U, 'N', 2, 'direction', 'backward');

% Plot the sets
figure()
hold on
plot(initial_cont2, 'color', [0/255 0/255 60/255])
plot(initial_cont1, 'color', [0/255 100/255 60/255])
plot(2, 0, 'o', 'MarkerFaceColor', 'auto', 'MarkerEdgeColor', 'k', 'MarkerSize', 8)
legend('Controller 2 Feasible Set', 'Controller 1 Feasible Set', 'Initial State [2 0]')
xlabel('x_1')
ylabel('x_2')
title('Feasible Initial States')



X0_1_3c = 'Set of feasible initial states for controller designed in Q3a';
X0_2_3c = 'Set of feasible initial states for controller designed in Q3b';

