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

    % Initialize occupancy grid
    cube_locs = [ [3,-8, 0]; [9, 0, 0]; [6, 6, 0] ].';
    % We take the transpose to be able to index properly
    % Grid i,j coordinates, as well as current height
    % (in stacking terms) of the cube
    cube_hold = [ [3,-8]; [5,-5]; [4, 0]; [9, 0]; [0, 4]; [6, 6] ].';

    occupancyGrid= createOccupancyGrid(cube_locs, cube_hold);

    % Try to rotate cube at (9,0)
    startPos = [225, 0, 45, 0];
    endPos = [225, 0, 45, -pi/2];

    vias = AstarSearch(startPos, endPos, occupancyGrid);
    Tend = 10;

    % Initialization
    % Set Servo 2-4 to be in position control mode (so that they hold still!)
    % goal_pos_all = [2048 2048 2048 3000];
    % for i=2:4
    %     % Disable Dynamixel torque to write settings to it
    %     write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 0);
    %     % Put actuator into Position Control Mode
    %     write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_OPERATING_MODE, 3);
    %     % Re-enable dynamixel torque
    %     write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 1);
    %     % set everything to 90 deg
    %     write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_GOAL_POSITION, goal_pos_all(i));
    % end

    curr_pos = zeros(1,4);
    curr_vel = zeros(1,4);
    curr_err = zeros(1,4);
    err_acc = zeros(1,4);

    for i=1:4
        curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
        curr_vel(i) = twos2decimal(read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY), 32);
        % curr_err(i) = 0;
        err_acc(i) = 0;
    end

    % Calculate via points
    vias = [curr_pos; vias];
    Tend = 10;

    % Interpolate between waypoints
    T = assignViaTimes(vias, Tend, 'dvel');
    coeffs = interpQuinticTraj(vias, T);
    figure
    plotQuinticInterp(vias, coeffs, T);

    err_vec = curr_err;  % velocity histories
    vel_vec = curr_vel;
    pos_vec = curr_pos;
    command_vel_vec = zeros(1,4);
    desired_vel_vec = zeros(1,4);
    time_vec = 0;

    start_time = now;
    curr_time = 0;
    
    % Store previous target segment
    prevTargetIdx = 1;
    
    last_seg = false;   % If we are homing on the last segment

    % while abs(curr_err) > 5 && curr_time < Tend
    while ~( last_seg && all( abs(curr_err) < 5 ) )
        % TODO figure out what to do if control has not converged by the time ending
        if curr_time < Tend
            [desiredVel, targetIdx] = sampleQuinticVel(coeffs, T, curr_time);
        else
            desiredVel = zeros(1,4);
            targetIdx = size(vias,1);
        end

        last_seg = targetIdx == size(vias, 1)
        curr_err
        
        % Check if new segment
        if targetIdx ~= prevTargetIdx
            disp("Reset error accumulator!")
            err_acc = 0;
        end
        prevTargetIdx = targetIdx;
        [jointVel, err_acc] = feedforwardPIcontrol(desiredVel, curr_err, err_acc);

        % Convert sampled joint velocity from ticks/s to rev/min. One unit = 0.229 RPM
        jointVel = round( (jointVel * 60 / 4096) / 0.229 );
        
        jointLimFlag = true;
        for i=1:4

            % Motor will clamp to velocity limits automatically
            curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
            
            if curr_pos(i) > servoLimits(i,2) || curr_pos(i) < servoLimits(i,1)
                fprintf("VIOLATED JOINT LIMITS ON SERVO %d! EXITING\n", i);
                jointLimFlag = false;
                break % STOP EXECUTION
            end
            
            curr_vel(i) = twos2decimal(read4ByteTxRx (port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY), 32 );
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_GOAL_VELOCITY, typecast(int32(jointVel(i)), 'uint32') );
        end

        if ~jointLimFlag 
            break 
        end

        curr_time = (now-start_time) * 24 * 60 * 60;
        curr_err = vias(targetIdx, :) - curr_pos;   % current goal position is that for targetIdx
        err_vec(end+1,:) = curr_err;
        vel_vec(end+1,:) = curr_vel;
        command_vel_vec(end+1,:) = jointVel;
        desired_vel_vec(end+1,:) = round( (desiredVel * 60 / 4096) / 0.229 );
        pos_vec(end+1,:) = curr_pos;
        time_vec(end+1) = curr_time;
    end

    % Use PI controller to make sure that controller actually gets where it's supposed to go?
    % TODO : If velocity limit is clipped, then we won't actually hit the target vias.
    % TODO : Add in some feedback control at the end to solve it

    % Ensure that final speed is always 0
    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_GOAL_VELOCITY, 0);
    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(2), params.ADDR_PRO_GOAL_VELOCITY, 0);
    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(3), params.ADDR_PRO_GOAL_VELOCITY, 0);
    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(4), params.ADDR_PRO_GOAL_VELOCITY, 0);
    % Disable torque
    % write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_TORQUE_ENABLE, 0);

    % Plot histories for each joint
    figure

    for joint=1:4
        subplot(4,3, 3*(joint-1)+1)
        plot(time_vec, err_vec(:,joint))
        title("Error history - Joint " + joint)
        grid on

        subplot(4,3, 3*(joint-1)+2)
        plot(time_vec, pos_vec(:,joint))
        hold on
        title("Position history - Joint " + joint)
        for i=1:size(vias,1)
            plot(T(i),vias(i,joint),'ro')
        end
        grid on

        subplot(4,3, 3*(joint-1)+3)
        hold on
        plot(time_vec, vel_vec(:,joint))
        plot(time_vec, desired_vel_vec(:, joint));
        plot(time_vec, command_vel_vec(:,joint))
        title("Commanded vs actual velocities - Joint " + joint)
        legend("Actual velocity", "Desired velocity", "Commanded velocity")
        grid on
        % sgtitle("Joint " + joint)
        hold off
    end

end % End checking that dynamixels have set up correctly

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