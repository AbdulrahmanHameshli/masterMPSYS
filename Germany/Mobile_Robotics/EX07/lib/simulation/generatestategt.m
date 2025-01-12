function [newStateGT, newDeltaWheelAngles] = generatestategt(lastStateGT, lastDeltaWheelAngles, path, istep, config)
    if config.MANUALCONTROL
        newDeltaWheelAngles = manualcontrolhandler(lastDeltaWheelAngles);
    else
        [xControl,vControl] = positioncontrol(lastStateGT,path(:,istep),1,config.DT,config.B);
        % Calculate angular wheel increments for the control sequence
        angleControl = config.DT*vControl ./ repmat([config.RL; config.RR],1,size(vControl,2));
        % Sum to to get the total wheel increment to reach the next via point
        newDeltaWheelAngles = sum(angleControl, 2);
    end
    % Use differential drive model to calculate the next gt state
    [newStateGT,~,~] = ododdforward(lastStateGT,zeros(3),newDeltaWheelAngles(1),newDeltaWheelAngles(2),config.BGT,config.RLGT,config.RRGT,config.KL,config.KR);
end

