function params = getDXLParams(port_num)

    % ---- Control Table Addresses ---- %%
    params.ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
    params.ADDR_PRO_GOAL_POSITION       = 116; 
    params.ADDR_PRO_GOAL_VELOCITY       = 104;

    params.ADDR_PRO_PRESENT_POSITION    = 132; 
    params.ADDR_PRO_PRESENT_VELOCITY    = 128;
    params.ADDR_PRO_OPERATING_MODE      = 11;
    params.ADDR_PRO_DRIVE_MODE          = 10;
    
    params.ADDR_PRO_PROFILE_VELOCITY    = 112;
    params.ADDR_PRO_PROFILE_ACCEL       = 108;
    params.ADDR_VEL_LIMIT = 44;
    
    params.ADDR_VEL_P_GAIN = 78;
    params.ADDR_VEL_I_GAIN = 76;

    params.ADDR_POS_P_GAIN = 84;
    params.ADDR_POS_I_GAIN = 82;
    params.ADDR_POS_D_GAIN = 80;

    % ---- Other Settings ---- %%
    
    % Protocol version
    params.PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel
    
    % Default setting
    params.DXL_LIST = [11,12,13,14,15];
    params.BAUDRATE                    = 1000000;
    params.DEVICENAME                  = 'COM10';       % Check which port is being used on your controller
    
    params.TORQUE_ENABLE               = 1;            % Value for enabling the torque
    params.TORQUE_DISABLE              = 0;            % Value for disabling the torque
    params.DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
    params.DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    params.DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

    params.ESC_CHARACTER               = 'e';          % Key for escaping loop

    params.COMM_SUCCESS                = 0;            % Communication Success result value
    params.COMM_TX_FAIL                = -1001;        % Communication Tx Failed

    % ----- SET MOTION LIMITS ----------- %%
    params.ADDR_MAX_POS = 48;
    params.ADDR_MIN_POS = 52;
    params.MAX_POS = 3400;
    params.MIN_POS = 600;
    % ---------------------------------- %%

end