function deltaWheelAngles = manualcontrolhandler(deltaWheelAngles)
deltaLeftWheelAngle = deltaWheelAngles(1);
deltaRightWheelAngle = deltaWheelAngles(2);
if waitforbuttonpress
    ctrlinput = upper(get(gcf,'CurrentCharacter'));

    turnStrength = 0.25;
    switch(ctrlinput)
        case 'W'

            deltaLeftWheelAngle = 3.0;
            deltaRightWheelAngle = 3.0;
        case 'S'
            deltaLeftWheelAngle = 0;
            deltaRightWheelAngle = 0;
        case 'A'
            if deltaLeftWheelAngle > deltaRightWheelAngle
                deltaLeftWheelAngle = deltaRightWheelAngle;
            end
            deltaLeftWheelAngle = deltaLeftWheelAngle - turnStrength;
            deltaLeftWheelAngle = min(deltaLeftWheelAngle, 3.5);
            deltaLeftWheelAngle = max(deltaLeftWheelAngle, -3.5);
            deltaRightWheelAngle = deltaRightWheelAngle + turnStrength;
            deltaRightWheelAngle = min(deltaRightWheelAngle, 3.5);
            deltaRightWheelAngle = max(deltaRightWheelAngle, -3.5);
        case 'D'
            if deltaLeftWheelAngle < deltaRightWheelAngle
                deltaRightWheelAngle = deltaLeftWheelAngle;
            end
            deltaLeftWheelAngle = deltaLeftWheelAngle + turnStrength;
            deltaLeftWheelAngle = min(deltaLeftWheelAngle, 3.5);
            deltaLeftWheelAngle = max(deltaLeftWheelAngle, -3.5);
            deltaRightWheelAngle = deltaRightWheelAngle - turnStrength;
            deltaRightWheelAngle = min(deltaRightWheelAngle, 3.5);
            deltaRightWheelAngle = max(deltaRightWheelAngle, -3.5);
        
        otherwise
            disp("using previous control - use w/a/s/d to steer or q to quit")
    end
end
deltaWheelAngles = [deltaLeftWheelAngle, deltaRightWheelAngle];

end

