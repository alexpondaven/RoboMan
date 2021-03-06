function jointParams = getJointParams()
% Returns joint configuration mapping.
% Servo bounds are expressed in terms of ticks

    % Put joint offset into each of the `deg2rad` terms for servo calibration on the day itself.

    joint_bounds = getServoLimits();    % Expressed in ticks
    D11_PARAMS = struct('LB', joint_bounds(1,1), ...
    'UB', joint_bounds(1,2), ...
    'invert', 1, ...
    'offset', pi + deg2rad(0));
    % Shoulder
    D12_PARAMS = struct('LB', joint_bounds(2,1), ...
    'UB', joint_bounds(2,2), ...
    'invert', -1, ...
    'offset', pi + deg2rad(0));
    % Elbow
    D13_PARAMS = struct('LB', joint_bounds(3,1), ...
    'UB', joint_bounds(3,2), ...
    'invert', -1, ...
    'offset', pi + deg2rad(0));

    % Wrist
    D14_PARAMS = struct('LB', joint_bounds(4,1), ...
    'UB', joint_bounds(4,2), ...
    'invert', -1, ...
    'offset', pi + deg2rad(0));

    % Gripper
    D15_PARAMS = struct('LB', joint_bounds(5,1), ...    % wide open
    'UB', joint_bounds(5,2), ...                        % fully closed
    'invert', 1, ...
    'offset', 0 + deg2rad(0));

    jointParams = [D11_PARAMS, D12_PARAMS, D13_PARAMS, D14_PARAMS, D15_PARAMS];

end