close all; clear; clc;

% Random seed (change in Ex05.1 c) + Ex05.2 b))
RNDSEED = 100;
% Sequence length K
NOBSERVATIONS = 100;    
% Note the coloring of the states: 1: blue, 2: green, 3: red

% Set seed and define some variables
rng("default");
rng(RNDSEED);
nobs = NOBSERVATIONS;  % alias for better code readability
nstates = 3;


%% HMM Parameters
% Transition matrix A : syntax (i,j) means from state i to state j
% Observation matrix E: syntax (i,j) means prob. that symbol j is observed in state i
A = [0.97  0.02  0.01;
     0.03  0.95  0.02;
     0.025  0.025  0.95];
E = [0.7  0.2  0.1;
     0.1  0.8  0.1;
     0.1  0.3  0.6];
% Priors: uniform priors over states
priors = 1/nstates*ones(nstates,1);

% Generate a random state and observation sequence given the HMM parameters 
[obs, states] = hmmgenerate(nobs,A,E);


%% --- Exercise 05.1: Forward-backward algorithm
% Forward pass

% Initialize forward probabilities (filtered)
alpha = zeros(nstates, nobs+1);
alpha(:, 1) = priors;

for i = 1:nobs
    Dk = diag(E(:, obs(i))); % 
    alpha(:, i+1) = Dk * A' * alpha(:, i);
    alpha(:, i+1) = alpha(:, i+1) / sum(alpha(:, i+1));
end

% Plot the filtered probabilities (forward pass)
figure;
time = 0:nobs;
plot(time, alpha', 'LineWidth', 2);
xlabel('Time');
ylabel('Probability');
title('Filtered Probabilities (Forward Algorithm)');
legend('State 1', 'State 2', 'State 3');
grid on;

% Backward pass

% Initialize backward probabilities
beta = zeros(nstates, nobs+1);
beta(:, end) = 1; % Initialize the last backward probabilities to 1

for k = nobs:-1:1
    Dk = diag(E(:, obs(k))); 
    beta(:, k) = A * Dk * beta(:, k+1);
    beta(:, k) = beta(:, k) / sum(beta(:, k));
end

% Plot the backward probabilities
figure;
time = 0:nobs;
plot(time, beta', 'LineWidth', 2);
xlabel('Time');
ylabel('Probability');
title('Backward Probabilities');
legend('State 1', 'State 2', 'State 3');
grid on;

% Compute smoothed probabilities (gamma)
gamma = alpha .* beta; 
gamma = gamma ./ sum(gamma, 1); 

% Plot the smoothed probabilities with the true states
figure;
hold on;
time = 0:nobs;
plot(time, gamma', 'LineWidth', 2); % Smoothed probabilities
stairs(0:nobs-1, states, '--k', 'LineWidth', 1.5); % True states as stairs
xlabel('Time');
ylabel('Probability / State');
title('Smoothed Probabilities with True States');
legend('State 1', 'State 2', 'State 3', 'True States');
grid on;
hold off;


%% --- Exercise 05.2: Localization (and path recovery) in a topological map
nobs = 1000;
nstates = 8;
sizeobs = 10;
% transition matrix A, sum(A,2) must be 1
A = [
    0.1, 0.6, 0.2, 0.1, 0,   0,   0,   0;    % SIR Lab
    0.1, 0.2, 0.5, 0.1, 0.1, 0,   0,   0;    % Campus Beach
    0.1, 0, 0.1, 0.5, 0.1, 0.1, 0.1, 0;      % Cafeteria Contrast
    0.1, 0.1, 0.2, 0.1, 0.5, 0,   0,   0;    % Makerspace
    0.1, 0,   0.1, 0.1, 0.1, 0.5, 0.1, 0;  % Library
    0,   0,   0,   0,   0.1, 0.1, 0.6, 0.2;  % Mensa
    0.1, 0,   0,   0,   0,   0,   0.1, 0.8   % Student Dorms
    0.5, 0.2, 0.1, 0.1, 0,   0,   0,   0.1;  % Gym
];

% emission matrix E, sum(E,2) must be 1
E = [
    0.4, 0.4, 0,   0,   0,   0.2,   0,   0,   0,   0;     % SIR Lab
    0.1,   0,   0.8, 0,   0,   0.1,   0,   0,   0,   0;   % Campus Beach
    0,   0.1,   0,   0.6, 0,   0,   0,   0.3,   0,   0;   % Cafeteria Contrast
    0,   0,   0,   0,   0.7, 0.2,   0,   0.1,   0,   0;   % Makerspace
    0,   0,   0,   0,   0,   0.5, 0.5, 0,   0,   0;       % Library
    0,   0,   0.1,   0,   0,   0,   0.8, 0,   0.1,   0;   % Mensa
    0,   0,   0,   0,   0,   0,   0,   0.7, 0,   0.3;     % Student Dorms
    0.1,   0,   0,   0,   0,   0,   0,   0,   0,   0.9;   % Gym
];

% labels for the emission matrix columns
landmarkLabels = {'Go2 Robot', 'Pepper Robot', 'Beach Chairs', 'Espresso Machine', ...
                  '3D Printers', 'Study Tables', 'Bookshelves', 'Buffet Station', ...
                  'Student Bikes', 'Workout Equipment'};

stateLabels = {'SIR Lab', 'Campus Beach', 'Cafeteria Contrast', ...
               'Makerspace', 'Library', 'Mensa', ...
               'Student Dorms', 'Gym'};

% Generate a random state and observation sequence given the HMM parameters 
[obs, states] = hmmgenerate(nobs,A,E);

n = nstates;
T = nobs;

% Initialize
viterbi = zeros(n, T); % Viterbi probabilities
backpointer = zeros(n, T); % Backpointer for path reconstruction
path = zeros(1, T); % Most likely state sequence

% Initialization (time step 1)
viterbi(:, 1) = log(E(:, obs(1))) + log(1/nstates); % Start with uniform priors

% Recursion
for t = 2:T
    for s = 1:n
        [viterbi(s, t), backpointer(s, t)] = max(viterbi(:, t-1) + log(A(:, s)));
        viterbi(s, t) = viterbi(s, t) + log(E(s, obs(t)));
    end
end

% Termination
[~, path(T)] = max(viterbi(:, :));




% 1. Ground truth states
figure;
plot(states, 'LineWidth', 2);
hold on;
plot(path, '--', 'LineWidth', 2);
legend('Ground Truth', 'Recovered Path');
xlabel('Time');
ylabel('State');
title('Ground Truth vs. Recovered Path');
grid on;

% 2. Confusion Matrix
confMatrix = confusionmat(states, path);
figure;
heatmap(stateLabels, stateLabels, confMatrix);
title('Confusion Matrix');
xlabel('Predicted States');
ylabel('True States');
