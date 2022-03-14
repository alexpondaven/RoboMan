function raw_angles = inverseKinDynamixel2(raw_angles)
% Function to convert raw angles in radians to Dynamixel servo ticks.

    PARAMS = getJointParams();
    
    % modify raw_angles
    for i=1:5
        % Modify raw_angles
        raw_angles(i) = raw_angles(i) * PARAMS(i).invert;     % invert if necessary (for shoulder, wrist, elbow)
        raw_angles(i) = raw_angles(i) + PARAMS(i).offset;

        % Convert to ticks
        raw_angles(i) = raw_angles(i) * 2048/pi;
        raw_angles(i) = cast(raw_angles(i), 'uint32');

        % Make sure raw_angles is in range described by PARAMS
        if raw_angles(i) > PARAMS(i).UB
            fprintf("Angle %d of servo %d over upper bound %d\n", raw_angles(i), i, PARAMS(i).UB)
            
            raw_angles(i) = PARAMS(i).UB;
        end
        if raw_angles(i) < PARAMS(i).LB
            fprintf("Angle %d of servo %d under lower bound %d\n", raw_angles(i), i, PARAMS(i).LB)
            
            raw_angles(i) = PARAMS(i).LB;
        end
        
    end

end