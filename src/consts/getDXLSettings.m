function dxlSettings = getDXLSettings()

    %% Dynamixel parameters
    dxlSettings.servoLimits = getServoLimits();    
    dxlSettings.velocityLimit = 20;     % 4.58rpm
    % Velocity PI Gains
    dxlSettings.DYN_VEL_P = 65;
    dxlSettings.DYN_VEL_I = 900;

    % Position PID Gains
    dxlSettings.DYN_POS_P = 400;
    dxlSettings.DYN_POS_I = 50;
    dxlSettings.DYN_POS_D = 0;

    % Profile velocity and acceleration values
    dxlSettings.DYN_PRO_VEL = 2048;
    dxlSettings.DYN_PRO_ACC = 4000;

end