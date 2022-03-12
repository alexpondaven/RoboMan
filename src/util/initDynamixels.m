function initDynamixels(port_num)
    % Initialize all dynamixels
    
    params = getDXLParams();
    
    servoLimits = getServoLimits();
    % VEL_LIMIT = 800;                            % ticks per second
    % velocityLimit = VEL_LIMIT/4096*60;          % Convert to rpm
    % velocityLimit = velocityLimit*0.229;        % convert to config settings
    % velocityLimit = cast(velocityLimit, 'int32');
    velocityLimit = 200;     % 5.73rpm

    for i=1:length(params.DXL_LIST)
        % Put actuator into Position Control Mode
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_OPERATING_MODE, 3);
        % Change this to velocity control mode (1)
        % Set max position limit
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MAX_POS, servoLimits(i,2));
        % Set min position limit
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_MIN_POS, servoLimits(i,1));
        % Set Dynamixel Velocity Limit
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_LIMIT, velocityLimit);
        % Set Dynamixel Velocity PI gains
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_P_GAIN, 65);
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_VEL_I_GAIN, 900);
        
        % Set Dynamixel Torque
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 1);


        % Set profile velocity - smoother motion
        % write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PROFILE_VELOCITY, 1024);
        % Profile acceleration

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

end