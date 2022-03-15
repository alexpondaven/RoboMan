function servoLimits = getServoLimits()
% Returns servo limits in terms of servo ticks.
% 5-array with tuple of (lb, ub)

    jointBounds = getJointBounds();
    servoLimits = ( jointBounds + [[ pi pi pi pi 0 ];[ pi pi pi pi 0 ]] )';
    % offset term for each servo

    servoLimits = cast(servoLimits.*(2048/pi), 'uint32');
end

