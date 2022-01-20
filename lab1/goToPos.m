function [pos_history, setpoint_history, time_vec] = goToPos(port_num, DXL_ID, set_point)
    ADDR_PRO_GOAL_POSITION       = 116; 
    ADDR_PRO_PRESENT_POSITION    = 132;
    COMM_SUCCESS = 0;
    PROTOCOL_VERSION            = 2.0; 
    DXL_MOVING_STATUS_THRESHOLD = 20;
    DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    
    THRESHOLD = 10;
    
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];
    index = 1;
    dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, set_point);

    pos_history = [];
    setpoint_history = [];
    time_vec = [];
    start = now;

    while (abs(dxl_present_position - set_point) > THRESHOLD)
        
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
        pos_history(end+1) = dxl_present_position;
        setpoint_history(end+1) = set_point;
        time_vec(end+1) = now - start;

        % Error handling
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('[ID:%03d] Position: %03d\n', DXL_ID, typecast(uint32(dxl_present_position), 'int32'));

        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end
end