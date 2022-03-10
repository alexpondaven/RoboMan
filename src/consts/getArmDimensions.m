function dimensions = getArmDimensions()
% Returns the dimensions of the links of the robot arm.
%
    dimensions.L1 = 128;
    dimensions.L2 = 24;
    dimensions.L3 = 124;
    dimensions.L4 = 126;           % Alter this parameter for end effector grip position
    dimensions.alpha_3 = atan(dimensions.L2/dimensions.L1);
    dimensions.R_3 = sqrt(dimensions.L2^2 + dimensions.L1^2);
end

