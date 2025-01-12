function transitionProbabilities = transitionmodel(map, state, control)

Pdesired  = 0.8;  % Probability that robot moves in desired direction
Pdiagonal = 0.05;   % Probability that robot moves in the two diagonal directions
Pstay     = 0.1;  % Probability of no motion, i.e. robot stays where it is
                   % NOTE: Pdesired + 2*Pdiagonal + Pstay must sum to 1

[nycells, nxcells] = size(map);
transitionProbabilities = zeros(nycells, nxcells);

% Determine correct indices given our motion range and map limits
if state(1) <= 1
  xirange = state(1):state(1)+1;
elseif state(1) >= nxcells
  xirange = state(1)-1:state(1);
else
  xirange = state(1)-1:state(1)+1;
end

if state(2) <= 1
  yirange = state(2):state(2)+1;
elseif state(2) >= nycells
  yirange = state(2)-1:state(2);
else
  yirange = state(2)-1:state(2)+1;
end

% Loop over 3x3 mask defined by motion range (or smaller at map boundaries)
for xi = xirange
  for yi = yirange

    deltax = xi - state(1);
    deltay = yi - state(2);
    switch control
      case 'N'
        if (deltax     ==  0 && deltay ==  1)
          transitionProbabilities(yi, xi) = Pdesired;
        elseif (deltax == -1 && deltay ==  1)
          transitionProbabilities(yi, xi) = Pdiagonal;
        elseif (deltax ==  1 && deltay ==  1)
          transitionProbabilities(yi, xi) = Pdiagonal;
        end
      case 'S'
        if (deltax     ==  0 && deltay == -1)
          transitionProbabilities(yi, xi) = Pdesired;
        elseif (deltax == -1 && deltay == -1)
          transitionProbabilities(yi, xi) = Pdiagonal;
        elseif (deltax ==  1 && deltay == -1)
          transitionProbabilities(yi, xi) = Pdiagonal;
        end
      case 'W'
        if (deltax     == -1 && deltay ==  0)
          transitionProbabilities(yi, xi) = Pdesired;
        elseif (deltax == -1 && deltay ==  1)
          transitionProbabilities(yi, xi) = Pdiagonal;
        elseif (deltax == -1 && deltay == -1)
          transitionProbabilities(yi, xi) = Pdiagonal;
        end
      case 'E'
        if (deltax     ==  1 && deltay ==  0)
          transitionProbabilities(yi, xi) = Pdesired;
        elseif (deltax ==  1 && deltay ==  1)
          transitionProbabilities(yi, xi) = Pdiagonal;
        elseif (deltax ==  1 && deltay == -1)
          transitionProbabilities(yi, xi) = Pdiagonal;
        end
    end
    % Case robot stays where it is
    if (deltax == 0 && deltay == 0)
      transitionProbabilities(yi, xi) = Pstay;
    end
  end
end
