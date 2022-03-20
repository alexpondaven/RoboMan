function dimensions = getArmDimensions()
% Returns the dimensions of the links of the robot arm.
%
    dimensions.L1 = 128;
    dimensions.L2 = 24;
    dimensions.L3 = 124;
    
    %% For Task 2 Gripper
    dimensions.L4 = 75 + 22.5;        % Alter this parameter for end effector grip position (27.5)
                                    % 75 is the distance to the gripper adapter
    dimensions.L5 = 10;

    %% For Task 3 Pen Holder
    dimensions.L4 = 75 + 22.5;
    dimensions.L5 = 0;

    dimensions.theta_go = atan2(dimensions.L5, dimensions.L4);
    dimensions.d_go = hypot(dimensions.L5, dimensions.L4);
    dimensions.alpha_3 = atan(dimensions.L2/dimensions.L1);
    dimensions.R_3 = sqrt(dimensions.L2^2 + dimensions.L1^2);
end

