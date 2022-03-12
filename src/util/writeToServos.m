function writeToServos(theta, mode, port_num)

    params = getDXLParams();
    DXL_LIST = params.DXL_LIST;
    PROTOCOL_VERSION = params.PROTOCOL_VERSION;

    if mode=="pos"
        TARGET = params.ADDR_PRO_GOAL_POSITION;
    elseif mode=="vel"
        TARGET = params.ADDR_PRO_GOAL_VELOCITY;
    else
        disp('unknown control mode')
        return
    end
    
    for i=1:length(DXL_LIST)
        fprintf("Servo %d | Reg %d | Val %d\n", DXL_LIST(i), TARGET, theta(i));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), TARGET, theta(i));
    end
    
end