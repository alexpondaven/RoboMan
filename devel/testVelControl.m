%% Code to test velocity control

%% Initialization
clc;
clear;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

params = getDXLParams();
port_num = portHandler(params.DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();
dxl_comm_result = params.COMM_TX_FAIL;           % Communication result
dxl_error = 0;                              % Dynamixel error

% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, params.BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Check for error in initializing dynamixels
if initDynamixels(port_num, 'vel') == 0
    %% DO STUFF HERE

    servoLimits = getServoLimits();
    velocityLimit = getDXLSettings().velocityLimit;

    % write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_TORQUE_ENABLE, 0);

    % Initialization
    % Set Servo 2-4 to be in position control mode (so that they hold still!)
    goal_pos_all = [2048 2048 2048 3000];
    for i=2:4
        % Disable Dynamixel torque to write settings to it
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 0);
        % Put actuator into Position Control Mode
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_OPERATING_MODE, 3);
        % Re-enable dynamixel torque
        write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 1);
        % set everything to 90
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_GOAL_POSITION, goal_pos_all(i));
    end

    goal_pos = goal_pos_all(1);    % For servo 1
    curr_pos = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_PRESENT_POSITION);
    curr_vel = twos2decimal(read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_PRESENT_VELOCITY), 32);
    curr_err = goal_pos - curr_pos;
    err_acc = 0;

    % Interpolate between waypoints
    vias = [curr_pos goal_pos_all(2:4); goal_pos_all(1:4); curr_pos goal_pos_all(2:4)];
    Tend = 10;
    T = assignViaTimes(vias, Tend, 'lin');
    coeffs = interpQuinticTraj(vias, T);
    figure
    plotQuinticInterp(vias, coeffs, T);

    err_vec(1) = curr_err;  % velocity histories
    vel_vec(1) = curr_vel;
    command_vel_vec(1) = 0;
    time_vec(1) = 0;

    start_time = now;
    curr_time = 0;
    % while abs(curr_err) > 5 && curr_time < Tend
    while curr_time < Tend
        % TODO figure out what to do if control has not converged by the time ending
        desiredVel = sampleQuinticVel(coeffs, T, curr_time);
        desiredVel = desiredVel(1);


        % Convert into RPM?
        [jointVel, err_acc] = feedforwardPIcontrol(desiredVel, curr_err, err_acc);

        % Convert sampled joint velocity from ticks/s to rev/min
        % one unit = 0.229 RPM
        jointVel = round( (jointVel * 60 / 4096) / 0.229 );

        % Clamp to velocity limits
        if jointVel > 0
            jointVel = min(jointVel, velocityLimit);
        else
            jointVel = max(jointVel, -velocityLimit);
        end

        curr_pos = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_PRESENT_POSITION);
        
        if curr_pos > servoLimits(1,2) || curr_pos < servoLimits(1,1)
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_GOAL_VELOCITY, 0);
            disp("VIOLATED JOINT LIMITS! EXITING")
            break % STOP EXECUTION
        end
        
        curr_vel = twos2decimal(read4ByteTxRx (port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_PRESENT_VELOCITY), 32 );
        
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_GOAL_VELOCITY, typecast(int32(jointVel), 'uint32') );
        
        curr_time = (now-start_time) * 24 * 60 * 60;
        curr_err = goal_pos - curr_pos;
        err_vec(end+1) = curr_err;
        vel_vec(end+1) = curr_vel;
        command_vel_vec(end+1) = jointVel;
        time_vec(end+1) = curr_time;

    end

    % Ensure that final speed is always 0
    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_GOAL_VELOCITY, 0);
    % Disable torque
    write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_TORQUE_ENABLE, 0);

    % Plot histories
    figure
    subplot(2,1,1)
    plot(time_vec, err_vec)
    title("Error history")
    grid on

    subplot(2,1,2)
    plot(time_vec, command_vel_vec)
    hold on
    plot(time_vec, vel_vec)
    title("Commanded vs actual velocities")
    legend("Commanded velocity", "Actual velocity")
    grid on
    hold off

end % End bug-free code

%% -- Dynamixel Cleanup Start -- %%
for i=1:length(params.DXL_LIST)
    % Disable Dynamixel Torque
    % write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 0);
    dxl_comm_result = getLastTxRxResult(port_num, params.PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, params.PROTOCOL_VERSION);
    if dxl_comm_result ~= params.COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(params.PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(params.PROTOCOL_VERSION, dxl_error));
    end
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);