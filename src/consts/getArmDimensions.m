function dimensions = getArmDimensions()
% Returns the dimensions of the links of the robot arm.
%
    dimensions.L1 = 128;
    dimensions.L2 = 24;
    dimensions.L3 = 124;
    dimensions.L4 = 110;           % Alter this parameter for end effector grip position
    dimensions.L5 = 10;
    dimensions.theta_go = atan2(dimensions.L5, dimensions.L4);
    dimensions.d_go = hypot(dimensions.L5, dimensions.L4);
    dimensions.alpha_3 = atan(dimensions.L2/dimensions.L1);
    dimensions.R_3 = sqrt(dimensions.L2^2 + dimensions.L1^2);
end

