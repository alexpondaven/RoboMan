%% Code to test drwaing fir task 3

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

    %Task 3 Drawing
    %get the current position 
    % might not need in traangle drawing
    % curr_pos = zeros(1,4);
    % for i=1:4
    %     curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    % end
    % % To be tested, the start position for the pen
    % startPos = [60, 200, 50, -pi/2];
    % currEndpointCoords = getCurrEndpointCoords(curr_pos);
    % via_paths = calcViaPoints(AStarWaypoints, occupancyGridVect);   % TODO define occupancyGridVect
    % coeff_paths = interpViaPoints(via_paths, true);


    servoLimits = getServoLimits();
    velocityLimit = getDXLSettings().velocityLimit;

    % Corners
    z=90;
    coords = [ 200 60  z
               200 140 z
               125 140 z
               200 60  z ];

     % interpolate lines between corners
     corners = [];
     numPoints = 10;
     for i=2:length(coords)
         corners = [corners; linearInterpolate(coords(i-1,:), coords(  i,:), numPoints) ];
     end

     GRIP_POS = deg2rad(232);

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
 
     %  calculate vias with adajustment to the gripper angle_thetaG  
     vias = curr_pos;
     for j=1:size(corners, 1)
         % corners(j,:)
         theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2, GRIP_POS);
         vias = [vias; theta(1:4)];
     end

      % Interpolate between waypoints
    [T, Tend] = assignViaTimes(vias, 'dvel');    % Tend no longer used
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
            desiredPos = sampleQuintic(coeffs, T, curr_time);
        else
            % Aim to be at final position with zero velocity
            desiredPos = vias(end,:);
            desiredVel = zeros(1,4);
            targetIdx = size(vias,1);
        end

        last_seg = targetIdx == size(vias, 1);

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
%         curr_err = vias(targetIdx, :) - curr_pos;   % current goal position is that for targetIdx
        curr_err = desiredPos - curr_pos; % Difference between sampled quintic position and measured position
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