function status = initDynamixels(port_num, mode)
    % Initialize all dynamixels
    
    params = getDXLParams();
    dxlSettings = getDXLSettings();

    %% Dynamixel parameters
    servoLimits = dxlSettings.servoLimits;    
    velocityLimit = dxlSettings.velocityLimit;     % 5.73rpm

    % Velocity PI Gains
    DYN_VEL_P = dxlSettings.DYN_VEL_P;
    DYN_VEL_I = dxlSettings.DYN_VEL_I;

    % Position PID Gains
    DYN_POS_P = dxlSettings.DYN_POS_P;
    DYN_POS_I = dxlSettings.DYN_POS_I;
    DYN_POS_D = dxlSettings.DYN_POS_D;

    % Profile velocity and acceleration values
    DYN_PRO_VEL = dxlSettings.DYN_PRO_VEL;
    DYN_PRO_ACC = dxlSettings.DYN_PRO_ACC;

    if mode=="vel"
        MODE(1:4) = [1 1 1 1];
    else
        MODE(1:4) = [3 3 3 3];   % position control
    end
    MODE(5) = 3;    % always put servo 5 into position control

    %% Write to servos
    for i=1:length(params.DXL_LIST)
        % Disable Dynamixel torque to write settings to it
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 0);

        % Put actuator into Position Control Mode
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_OPERATING_MODE, MODE(i));
        
        % Set max position limit
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MAX_POS, servoLimits(i,2));
        % Set min position limit
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MIN_POS, servoLimits(i,1));
        
        % Set Dynamixel Velocity Limit
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_LIMIT, velocityLimit);
        
        % Set Dynamixel Velocity PI gains
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_P_GAIN, DYN_VEL_P);
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_I_GAIN, DYN_VEL_I);
        
        % Set Dynamixel Position PI gains
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_P_GAIN, DYN_POS_P);
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_I_GAIN, DYN_POS_I);
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_D_GAIN, DYN_POS_D);

        % Set profile velocity - smoother motion
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PROFILE_VELOCITY, DYN_PRO_VEL);
        % Profile acceleration
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PROFILE_ACCEL, DYN_PRO_ACC);

        % Set Dynamixel Torque
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 1);

        dxl_comm_result = getLastTxRxResult(port_num, params.PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, params.PROTOCOL_VERSION);
        
        if dxl_comm_result ~= params.COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(params.PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(params.PROTOCOL_VERSION, dxl_error));
        else
            fprintf('Dynamixel %d has been successfully connected \n', i);
        end
    end

    status = 0;

    % Read from servos
    for i=1:length(params.DXL_LIST)
        % Disable Dynamixel torque to write settings to it

        % Put actuator into Position Control Mode
        ModeCheck = MODE(i)==read1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_OPERATING_MODE);
        
        % Set max position limit
        PosUBCheck = servoLimits(i,2)==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MAX_POS, servoLimits(i,2));
        % Set min position limit
        PosLBCheck = servoLimits(i,1)==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MIN_POS, servoLimits(i,1));
        
        % Set Dynamixel Velocity Limit
        VelLimCheck = velocityLimit==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_LIMIT);
        
        % Set Dynamixel Velocity PI gains
        velPCheck = DYN_VEL_P==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_P_GAIN);
        velICheck = DYN_VEL_I==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_I_GAIN);
        
        % Set Dynamixel Position PI gains
        posPCheck = DYN_POS_P==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_P_GAIN);
        posICheck = DYN_POS_I==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_I_GAIN);
        posDCheck = DYN_POS_D==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_D_GAIN);

        % Set profile velocity - smoother motion
        proVelCheck = DYN_PRO_VEL==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PROFILE_VELOCITY);
        % Profile acceleration
        proAccCheck = DYN_PRO_ACC==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PROFILE_ACCEL);

        CheckArr = [ModeCheck PosUBCheck PosLBCheck VelLimCheck velPCheck velICheck posPCheck posICheck posDCheck proVelCheck proAccCheck];

        if any(CheckArr==0)
            fprintf("Servo %d values not written correctly.\n", params.DXL_LIST(i));
            CheckArr    % Display error codes
            status = -1;    % error code
        end

    end

end