function observationProbability = observationmodel(map, state, observations)
    % True positive probability: The measurement probability when there is
    % something to measure (e.g. wall, obstacle)
    probabilityMeasure = 0.8;
    probabilityNotMeasure = 1 - probabilityMeasure; % False negative probability

    % Evaluate cell occupancy for North, South, West, East directions
    cellNorth = 0;
    cellSouth = 0;
    cellWest  = 0;
    cellEast  = 0;

    if (state(1)-1 > 0)
        cellWest = map(state(2), state(1)-1);  % West neighbor
    end
    if (state(1)+1 <= size(map,2))
        cellEast = map(state(2), state(1)+1);  % East neighbor
    end
    if (state(2)+1 <= size(map,1))
        cellNorth = map(state(2)+1, state(1)); % North neighbor
    end
    if (state(2)-1 > 0)
        cellSouth = map(state(2)-1, state(1)); % South neighbor
    end

    % Compute the observation probability
    observationProbability = 1;  % Initialize probability

    % Compare observations with the actual occupancy (1 for obstacle, 0 for free)
    if observations(1) == 1  % North bumper hit
        observationProbability = observationProbability * (cellNorth * probabilityMeasure + (1 - cellNorth) * probabilityNotMeasure);
    else  % No hit North
        observationProbability = observationProbability * ((1 - cellNorth) * probabilityMeasure + cellNorth * probabilityNotMeasure);
    end

    if observations(2) == 1  % South bumper hit
        observationProbability = observationProbability * (cellSouth * probabilityMeasure + (1 - cellSouth) * probabilityNotMeasure);
    else  % No hit South
        observationProbability = observationProbability * ((1 - cellSouth) * probabilityMeasure + cellSouth * probabilityNotMeasure);
    end

    if observations(3) == 1  % West bumper hit
        observationProbability = observationProbability * (cellWest * probabilityMeasure + (1 - cellWest) * probabilityNotMeasure);
    else  % No hit West
        observationProbability = observationProbability * ((1 - cellWest) * probabilityMeasure + cellWest * probabilityNotMeasure);
    end

    if observations(4) == 1  % East bumper hit
        observationProbability = observationProbability * (cellEast * probabilityMeasure + (1 - cellEast) * probabilityNotMeasure);
    else  % No hit East
        observationProbability = observationProbability * ((1 - cellEast) * probabilityMeasure + cellEast * probabilityNotMeasure);
    end
end
