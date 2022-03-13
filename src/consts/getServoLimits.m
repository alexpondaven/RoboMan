function servoLimits = getServoLimits()
% Returns servo limits in angle
% 5-array with tuple of (lb, ub)
    servoLimits(1, :) = [deg2rad(85), deg2rad(275)]; 
    servoLimits(2, :) = [deg2rad(65), deg2rad(280)]; 
    servoLimits(3, :) = [deg2rad(65), deg2rad(265)]; 
    servoLimits(4, :) = [deg2rad(80), deg2rad(300)]; 
    servoLimits(5, :) = [deg2rad(90), deg2rad(232)]; 

    servoLimits = cast(servoLimits.*(2048/pi), 'uint32');
end

