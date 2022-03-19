function retCode = setGripperPos(open, port_num)
% Sets gripper pose to desired position (open/closed)
% Blocks until the desired motion has been completed.
%
% Args:
% open     : (bool) Gripper position, true=open, closed=false
% port_num : value generated from main script
%
% Returns:
% 0 on successful function exit
    
    params = getDXLParams();
    if open==true
        targetAngle = 1024;  % claws wide open
    else
        % gripping the cube
        % targetAngle = 2435;     % with default gripper
        targetAngle = 1740;     % For CUBE GRIPPER
        % targetAngle = 1840 ;     % For DRAWING GRIPPER
    end

    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_PRO_GOAL_POSITION, targetAngle);
    
    curr_pos = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_PRO_PRESENT_POSITION);
    while( abs(curr_pos - targetAngle) > 2 )
        curr_pos = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_PRO_PRESENT_POSITION);
    end

    retCode = 0;

end