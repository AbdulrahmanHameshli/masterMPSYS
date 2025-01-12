function xnew = getnextstate(map, xgt, ctrlstr)

xnew = xgt;

% obtain all transition probabilities covering motions over the complete map
transition_probability = transitionmodel(map, xgt, ctrlstr);

% available motion range
xmin = xgt(1)-1; % W
xmax = xgt(1)+1; % E
ymin = xgt(2)-1; % N
ymax = xgt(2)+1; % S

% Inverse transform sampling: loop over the domain (here: available motion range)
% until cumulative probability is higher than a sample from uniform distribution
validmotion = false;
while ~validmotion

  minprob = rand(1);
  cumprob = 0;
  found = false;
  for xi = xmin:xmax
    for yi = ymin:ymax
      if ~found
        cumprob = cumprob + transition_probability(yi, xi);

        if cumprob > minprob
          xnew = [xi, yi];  % return with new position
          found = true;
        end
      end
    end
  end

  if map(xnew(2),xnew(1)) == 0
    validmotion = true;
  end
end