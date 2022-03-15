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

    %% Write to servos 1-4 (position control)
    for i=1:4
        % Disable Dynamixel torque to write settings to it
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 0);

        % Put actuator into Control Mode
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_OPERATING_MODE, MODE(i));
        
        if mode=="vel"
            % Set Dynamixel Velocity PI gains
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_P_GAIN, DYN_VEL_P);
            
            % Set Dynamixel Velocity Limit
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_LIMIT, velocityLimit);
            
            % Ensure that goal velocity is 0
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_GOAL_VELOCITY, 0);
        else
            % Set max position limit
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MAX_POS, servoLimits(i,2));
            % Set min position limit
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MIN_POS, servoLimits(i,1));
            
            % Set Dynamixel Position PI gains
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_P_GAIN, DYN_POS_P);
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_I_GAIN, DYN_POS_I);
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_D_GAIN, DYN_POS_D);

            % Set profile velocity - smoother motion
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PROFILE_VELOCITY, DYN_PRO_VEL);
            % Profile acceleration
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PROFILE_ACCEL, DYN_PRO_ACC);
        end

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

    %% Talk to Servo 5 (always position control mode)
    write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_PRO_TORQUE_ENABLE, 0);
    write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_PRO_OPERATING_MODE, 3);
    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_MAX_POS, servoLimits(5,2));
    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_MIN_POS, servoLimits(5,1));
    write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_PRO_TORQUE_ENABLE, 1);
    dxl_comm_result = getLastTxRxResult(port_num, params.PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, params.PROTOCOL_VERSION);
    if dxl_comm_result ~= params.COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(params.PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(params.PROTOCOL_VERSION, dxl_error));
    else
        fprintf('Dynamixel %d has been successfully connected \n', 5);
    end
    status = 0;

    % Read from servos 1-4
    for i=1:4
        % Put actuator into Position Control Mode
        ModeCheck = read1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_OPERATING_MODE);
        ModeCheck = ModeCheck==MODE(i);
        
        if mode=="vel"
            % Set Dynamixel Velocity Limit
            VelLimCheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_LIMIT);
            VelLimCheck = velocityLimit==VelLimCheck;
            
            % Set Dynamixel Velocity PI gains
            velPCheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_P_GAIN);
            velPCheck = DYN_VEL_P==velPCheck;

            CheckArr = [ModeCheck 1 1 VelLimCheck velPCheck 1 1 1 1 1];
        else
            % Set max position limit
            PosUBCheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MAX_POS);
            PosUBCheck = servoLimits(i,2)==PosUBCheck;
            % Set min position limit
            PosLBCheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MIN_POS);
            PosLBCheck = servoLimits(i,1)==PosLBCheck;

            % Set Dynamixel Position PI gains
            posPCheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_P_GAIN);
            posPCheck = DYN_POS_P==posPCheck;
            
            posICheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_I_GAIN);
            posICheck = DYN_POS_I==posICheck;
            posDCheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_POS_D_GAIN);
            posDCheck = DYN_POS_D==posDCheck;

            % Set profile velocity - smoother motion
            proVelCheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PROFILE_VELOCITY);
            proVelCheck = DYN_PRO_VEL==proVelCheck;
            % Profile acceleration
            proAccCheck = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PROFILE_ACCEL);
            proAccCheck = DYN_PRO_ACC==proAccCheck;

            CheckArr = [ModeCheck PosUBCheck PosLBCheck 1 1 posPCheck posICheck posDCheck proVelCheck proAccCheck];
        end

        if any(CheckArr==0)
            fprintf("Servo %d values not written correctly.\n", params.DXL_LIST(i));
            CheckArr    % Display error codes
            status = -1;    % error code
        end
    end

    %% Read from Servo 5
    ModeCheck = 3==read1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_PRO_OPERATING_MODE);
    PosUBCheck = servoLimits(5,2)==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_MAX_POS);
    PosLBCheck = servoLimits(5,1)==read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_MIN_POS);
    if any([ModeCheck, PosUBCheck, PosLBCheck] == 0)
        fprintf("Servo 5 values not written correctly.\n");
        [ModeCheck, PosUBCheck, PosLBCheck]
        status = -1;
    end

end