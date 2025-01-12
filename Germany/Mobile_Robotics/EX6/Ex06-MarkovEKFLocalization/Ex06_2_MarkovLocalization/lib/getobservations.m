function observations = getobservations(map, state)
observations = [0, 0, 0, 0];

% Sampling setup
minprob = rand(1);

% Inverse transform sampling: loop over the domain (here: binary numbers from 0-15
% that correspond to all possible bumper measurements) until cumulative probability 
% is higher than a sample from uniform distribution
cumprob = 0;
for iobs = 0:15
  %obs     = int2bit(iobs,4)';           % Matlab only
  obs     = double(dec2bin(iobs,4))-48;  % Octave needs this trick, it doesn't know int2bit 
  cumprob = cumprob + observationmodel(map, state, obs);
  
  if cumprob > minprob
    observations = obs;
    return;
  end
end


