    % Mobile Robotics Exercise
    %
    %% 
    % Markov Localization
    %
    % Main Script
    
    %% Prepare session
    clear;
    addpath lib/
    FONTSIZE = 14;
    MAPCOLOR = [.3 .5 .6];
    
    % Load map
    gridMap = load('officemap.txt'); % Can also be 'officemap.txt'
    gridMap = flipud(gridMap);  % To design map.txt in figure orientation
    [nRows, nCols] = size(gridMap);
    map.grid = gridMap;
    
    % Set robot to (arbitray) initial ground truth position
    state = [7, 2];
    
    % Initialize belief (we ignore cells occupied by the map obtacles)
    uniformBeliefProbability = 1/sum(sum(gridMap));
    map.belief = (1-gridMap)*uniformBeliefProbability;
    
    % Initialize observations (we assume a 4 quadrant bumper sensor
    % and measurements being transformed into global map frame)
    observations = [0 0 0 0];
    
    % Prepare map figure
    figTruth = subplot(1, 4, 1);
    % Plot map
    imagesc(gridMap);
    colormap(figTruth,flipud(gray));
    axis equal tight; hold on;
    set(gca,'YDir','normal');  % Flip y-axis to regular direction
    mesh(0.5:nCols+0.5,0.5:nRows+0.5,zeros([nRows nCols]+1),...
      'FaceColor','none','EdgeColor','k');
    set(title('Robot position (ground truth)'),'Fontsize',FONTSIZE);
    % Plot robot
    [hRobot, hObservation] = drawrobot(state, observations);
    
    % Prepare transition model figure
    figTransition = subplot(1, 4, 2);
    imagesc(zeros(size(map.belief)));
    colormap(figTransition,[1 1 1]);  % colormap for initial transition model distribution
    axis equal tight;
    set(gca,'YDir','normal');  % Flip y-axis to regular direction
    set(title('Transition model'),'Fontsize',FONTSIZE);
    
    % Prepare belief prediction figure
    figPrediction = subplot(1, 4, 3);
    imagesc(zeros(size(map.belief)));
    colormap(figPrediction,[1 1 1]);  % colormap for initial transition model distribution
    axis equal tight;
    set(gca,'YDir','normal');  % Flip y-axis to regular direction
    set(title('Predicted belief'),'Fontsize',FONTSIZE);
    
    % Prepare belief figure
    figBelief = subplot(1, 4, 4);
    % Define colormap with one specific color for the grid map
    cmap = flipud(gray(64));
    cmap(end+1,:) = MAPCOLOR;
    % Overlay initial belief
    imagesc(map.belief);
    colormap(figBelief,[MAPCOLOR;0.5 0.5 0.5]); % colormap for initial uniform belief 
    axis equal tight;
    set(gca,'YDir','normal');  % Flip y-axis to regular direction
    set(title('Updated belief'),'Fontsize',FONTSIZE);
    
    % Instruct user how to control robot
    disp('Move robot with W, A, S, D, and Q for quit.');
    
    
    %% Main loop
    running = true;
    while running
    
      % Simple way to control scipt via keyboard without GUIs and callbacks
      inputok = true;
      if waitforbuttonpress
        ctrlinput = upper(get(gcf,'CurrentCharacter'));
    
        switch(ctrlinput)
          case 'W'
            control = 'N';  % North direction
          case 'S'
            control = 'S';  % South direction
          case 'A'
            control = 'W';  % West direction
          case 'D'
            control = 'E';  % East direction
          case 'Q'
            running = false;
            continue;
          otherwise
            inputok = false;
            continue;
        end
    
        % Delete previous robot position and observations from figure
        subplot(figTruth);
        if exist('hRobot','var')
          delete(hRobot);
        end
        if exist('hObservation','var')
          delete(hObservation);
        end
        statePrevious = state;   % Save for transition model plot
    
        %% --------------------------- FILTERING ------------------------------
    
% Retrieve new robot position according to our transition model
state = getnextstate(map.grid, state, control);

% Obtain current observations according to our observation model
observations = getobservations(map.grid, state);

% Predict robot position belief    
% TODO
% Predict robot position belief using inline transition model
beliefPrediction = zeros(size(map.belief));

% Transition model probabilities
p_desired = 0.8;   % Probability of moving to the desired cell
p_stay = 0.1;      % Probability of staying in the current cell
p_diagonal = 0.15 / 4; % Probability of diagonal moves (equally divided)

for i = 1:size(map.grid, 1)
    for j = 1:size(map.grid, 2)
        if map.grid(i, j) == 0  % Only consider free cells
            % Sum over all possible previous states
            for u = -1:1
                for v = -1:1
                    prev_i = i - u;  % Previous row
                    prev_j = j - v;  % Previous column
                    if prev_i > 0 && prev_j > 0 && prev_i <= size(map.grid,1) && prev_j <= size(map.grid,2)
                        % Determine transition probability based on control input
                        if u == 0 && v == 0  % Stay in place
                            transitionProb = p_stay;
                        elseif abs(u) == 1 && abs(v) == 1  % Diagonal move
                            transitionProb = p_diagonal;
                        elseif (u == 0 && v ~= 0 && control == 'E') || ...
                               (u == 0 && v ~= 0 && control == 'W') || ...
                               (v == 0 && u ~= 0 && control == 'N') || ...
                               (v == 0 && u ~= 0 && control == 'S')  % Desired move
                            transitionProb = p_desired;
                        else
                            transitionProb = 0;  % Invalid move
                        end
            
                        % Add contribution from previous belief
                        beliefPrediction(i, j) = beliefPrediction(i, j) + ...
                            transitionProb * map.belief(prev_i, prev_j);
                    end
                end
            end

        end
    end
end


updatedBelief = zeros(size(map.belief));

for i = 1:size(map.grid, 1)
    for j = 1:size(map.grid, 2)
        if map.grid(i, j) == 0  % Only consider free cells
            % Call observationmodel to compute the observation probability
            observationProb = observationmodel(map.grid, [j, i], observations);

            % Update belief: predicted belief * observation probability
            updatedBelief(i, j) = beliefPrediction(i, j) * observationProb;
        end
    end
end

% Normalize the belief probabilities to sum to 1
normalizer = sum(updatedBelief, 'all');
if normalizer > 0
    map.belief = updatedBelief / normalizer;
else
    map.belief = updatedBelief; % Avoid division by zero if all probabilities are zero
end

        %% --------------------------------------------------------------------
    
        % Plot new ground truth robot position and observations
        subplot(figTruth);
        [hRobot, hObservation] = drawrobot(state, observations);
    
        % Plot transition model
        subplot(figTransition);
        transitionProbability = transitionmodel(map.grid, statePrevious, control);
        imagesc(transitionProbability);
        colormap(figTransition,flipud(gray(64)));
        axis equal tight;
        set(gca,'YDir','normal');  % Flip y-axis to regular direction
        set(title('Transition model'),'Fontsize',FONTSIZE);
    
        % Plot predicted belief
        subplot(figPrediction);
        imagesc(beliefPrediction);
        colormap(figPrediction,flipud(gray(64)));
        axis equal tight;
        set(gca,'YDir','normal');  % Flip y-axis to regular direction
        set(title('Predicted belief'),'Fontsize',FONTSIZE);
    
        % Plot updated belief
        subplot(figBelief);
        mapLayered = map.belief;
        maxBelief  = max(max(map.belief));
        mapLayered(map.grid == 1) = maxBelief+2*maxBelief/size(cmap,1); % compute value always unique for map color 
        imagesc(mapLayered);
        colormap(figBelief,cmap);
        axis equal tight;
        set(gca,'YDir','normal');  % Flip y-axis to regular direction
        set(title('Updated belief'),'Fontsize',FONTSIZE);
    
        % Status info
        fprintf('control input: %s, observations: %i %i %i %i, ground truth robot: x = %i, y = %i\n',...
          control, observations(1), observations(2), observations(3), observations(4),...
          state(1), state(2));
      end
    end
    
    

