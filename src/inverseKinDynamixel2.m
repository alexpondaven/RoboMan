function raw_angles = inverseKinDynamixel2(raw_angles)
% Function to convert raw angles in radians to Dynamixel servo ticks.

    joint_bounds = getServoLimits();

    % Hip
    D11_PARAMS = struct('LB', joint_bounds(1,1), ...
                 'UB', joint_bounds(1,2), ...
                 'invert', 1, ...
                 'offset', pi);
    % Shoulder
    D12_PARAMS = struct('LB', joint_bounds(2,1), ...
                 'UB', joint_bounds(2,2), ...
                 'invert', -1, ...
                 'offset', pi);
    % Elbow
    D13_PARAMS = struct('LB', joint_bounds(3,1), ...
                 'UB', joint_bounds(3,2), ...
                 'invert', -1, ...
                 'offset', pi);

    % Wrist
    D14_PARAMS = struct('LB', joint_bounds(4,1), ...
                 'UB', joint_bounds(4,2), ...
                 'invert', -1, ...
                 'offset', pi);
    
    % Gripper
    D15_PARAMS = struct('LB', joint_bounds(5,1), ...       % wide open
                 'UB', joint_bounds(5,2), ...            % fully closed
                 'invert', 1, ...
                 'offset', 0);
    
    PARAMS = [D11_PARAMS, D12_PARAMS, D13_PARAMS, D14_PARAMS, D15_PARAMS];
    % modify raw_angles
    for i=1:5
        % Modify raw_angles
        raw_angles(i) = raw_angles(i) * PARAMS(i).invert;     % invert if necessary (for shoulder, wrist, elbow)
        raw_angles(i) = raw_angles(i) + PARAMS(i).offset;

        % Make sure raw_angles is in range described by PARAMS
        if raw_angles(i) > PARAMS(i).UB
            fprintf("Angle %04d of servo %d over upper bound\n", raw_angles(i), i)
            
            raw_angles(i) = PARAMS(i).UB;
        end
        if raw_angles(i) < PARAMS(i).LB
            fprintf("Angle %04d of servo %d under lower bound\n", raw_angles(i), i)
            
            raw_angles(i) = PARAMS(i).LB;
        end
        
        % Convert to ticks
        raw_angles(i) = raw_angles(i) * 2048/pi;
    end

end