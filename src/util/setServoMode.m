function setServoMode(mode, port_num)
    % Sets the servos to be in velocity or position control mode.

    params = getDXLParams();

    if mode=="pos"
        TARGET = 3; % position control enum
    elseif mode=="vel"
        TARGET = 1; % vel control enum
    else
        disp('unknown control mode')
        return
    end
    
    
    for i=1:length(params.DXL_LIST)
        fprintf("Set servo %d to %s mode (%d)\n", params.DXL_LIST(i), mode, TARGET);
        % Turn off torque to change drive mode
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 0);
        
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_OPERATING_MODE, TARGET);
        % reenable torque
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 1);
    end
        
end