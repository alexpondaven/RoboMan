function retCode = mainServoLoopT4(coeffs, T, Tend, port_num, isPlot, vias, theta5)
% Main control loop for servo control.
% Handles low-level control for the servos to get from one position to another.
% The coefficients should already have been calculated previously, 
% so this is just making the servos dance to the choreography.
% 
% ARGS
% coeffs   : Coefficients for a single set of via points, calculated earlier from interpQuniticTraj
% T        : Timestamps for each via point as calculated from assignViaTraj
% Tend     : (float) Total planned duration for the motion
% port_num : port handler object to communicate with servos
% isPlot   : (bool) Whether or not to come up with debugging plot data
% vias     : vector of via points (in servo ticks) for each servo`
% TODO: remove vias for production code; only used for plotting
% 
% RETURNS
% error code if joint limits are reached

params = getDXLParams();
servoLimits = getServoLimits();
velocityLimit = getDXLSettings().velocityLimit;
COMM_SUCCESS = 0;

groupReadVelNum = groupSyncRead(port_num, params.PROTOCOL_VERSION, params.ADDR_PRO_PRESENT_VELOCITY, 4);
groupReadPosNum = groupSyncRead(port_num, params.PROTOCOL_VERSION, params.ADDR_PRO_PRESENT_POSITION, 4);
groupWriteNum = groupSyncWrite(port_num, params.PROTOCOL_VERSION, params.ADDR_PRO_GOAL_VELOCITY, 4);
for i=1:4
    if ~groupSyncReadAddParam(groupReadVelNum, params.DXL_LIST(i))
        fprintf("Sync read registration for curr_vel(%d) failed\n", i);
        return
    end

    if ~groupSyncReadAddParam(groupReadPosNum, params.DXL_LIST(i))
        fprintf("Sync read registration for curr_pos(%d) failed\n", i);
        return
    end
end

% Check for errors
groupSyncReadTxRxPacket(groupReadPosNum);
if getLastTxRxResult(port_num, params.PROTOCOL_VERSION) ~= COMM_SUCCESS
    disp("Error in position read");
end
groupSyncReadTxRxPacket(groupReadVelNum);
if getLastTxRxResult(port_num, params.PROTOCOL_VERSION) ~= COMM_SUCCESS
    disp("Error in position read");
end

% Initialise some variables
curr_pos = zeros(1,4);
curr_vel = zeros(1,4);

% Readout values
for i=1:4
    curr_pos(i) = groupSyncReadGetData(groupReadPosNum, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION, 4);
    curr_vel(i) = twos2decimal(groupSyncReadGetData(groupReadVelNum, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY, 4), 32);
end

currPosSetpoint = sampleQuintic(coeffs, T, 0);
curr_err = currPosSetpoint(:,1:4) - curr_pos;    % difference from desired start point at T=0 and current point
err_acc = curr_err;

if isPlot
    err_vec = curr_err;  % velocity histories
    vel_vec = curr_vel;
    pos_vec = curr_pos;
    command_vel_vec = zeros(1,4);
    desired_vel_vec = zeros(1,4);
    time_vec = 0;
end

start_time = now;
curr_time = 0;
prev_err = curr_err;

% Store previous target segment
last_seg = false;   % If we are homing on the last segment

retCode = 0;

while ~( last_seg && all( abs(curr_err) < 2 ) )
% % for testing
% while 1
    % if (now-start_time)*24*60*60 > 6
    %     break
    % end

    % TODO figure out what to do if control has not converged by the time ending
    if curr_time < Tend
        [desiredVel, targetIdx] = sampleQuinticVel(coeffs, T, curr_time);
        desiredPos = sampleQuintic(coeffs, T, curr_time);
    else
        % Aim to be at final position with zero velocity
        desiredPos = sampleQuintic(coeffs, T, Tend);
        desiredVel = zeros(1,4);
        targetIdx = size(T,2);
    end

    % Write current
    currTheta5Tick = theta5(targetIdx);
    fprintf("%0.2f, %d\n", currTheta5Tick, targetIdx);
    write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(5), params.ADDR_PRO_GOAL_POSITION, currTheta5Tick);

    last_seg = targetIdx == size(T,2);
    [jointVel, err_acc] = feedforwardPIcontrol2(desiredVel, curr_err, prev_err, err_acc);
    prev_err = curr_err;

    % Convert sampled joint velocity from ticks/s to rev/min. One unit = 0.229 RPM
    jointVel = round( (jointVel * 60 / 4096) / 0.229 );
    desiredVelScaled = round( (desiredVel * 60 / 4096) / 0.229 );

    % clamp jointVel
    jointVel = min(jointVel,velocityLimit);
    jointVel = max(jointVel,-velocityLimit);
    
    % fprintf("curr_pos: %04d | %04d | %04d | %04d\n", curr_pos(1), curr_pos(2), curr_pos(3), curr_pos(4));
    % fprintf("desiredPos: %.1f | %.1f | %.1f | %.1f\n", desiredPos(1), desiredPos(2), desiredPos(3), desiredPos(4));
    % fprintf("curr_err: %.1f | %.1f | %.1f | %.1f\n", curr_err(1), curr_err(2), curr_err(3), curr_err(4));
    % fprintf("curr_vel: %04d | %04d | %04d | %04d\n", curr_vel(1), curr_vel(2), curr_vel(3), curr_vel(4));
    % fprintf("desiredVel: %.1f | %.1f | %.1f | %.1f\n", desiredVelScaled(1), desiredVelScaled(2), desiredVelScaled(3), desiredVelScaled(4));
    % fprintf("jointVel: %04d | %04d | %04d | %04d\n\n", jointVel(1), jointVel(2), jointVel(3), jointVel(4));
    % startLoopTime = now;

    % Read motor values
    groupSyncReadTxRxPacket(groupReadPosNum);
    % Check for errors
    if getLastTxRxResult(port_num, params.PROTOCOL_VERSION) ~= COMM_SUCCESS
        disp("Error in position read");
    end
    groupSyncReadTxRxPacket(groupReadVelNum);
    % Check for errors
    if getLastTxRxResult(port_num, params.PROTOCOL_VERSION) ~= COMM_SUCCESS
        disp("Error in position read");
    end
    % Read and write values
    for i=1:4
        if groupSyncReadIsAvailable(groupReadPosNum, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION, 4)
            curr_pos(i) = groupSyncReadGetData(groupReadPosNum, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION, 4);
        else
            fprintf("curr_pos(%d) not avail\n", i);
        end

        if groupSyncReadIsAvailable(groupReadVelNum, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY, 4)
            curr_vel(i) = twos2decimal(groupSyncReadGetData(groupReadVelNum, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY, 4), 32);
        else
            fprintf("curr_vel(%d) not avail\n", i);
        end
    
        if curr_pos(i) > servoLimits(i,2) || curr_pos(i) < servoLimits(i,1)
            fprintf("VIOLATED JOINT LIMITS ON SERVO %d! EXITING\n", i);
    
            retCode = -1;
            for motor_idx=1:4
                groupSyncWriteAddParam( groupWriteNum, params.DXL_LIST(i), 0, 4 );
            end
            groupSyncWriteTxPacket(groupWriteNum);
            groupSyncWriteClearParam(groupWriteNum);
            
            break % STOP EXECUTION
        end
        groupSyncWriteAddParam( groupWriteNum, params.DXL_LIST(i), typecast(int32(jointVel(i)), 'uint32'), 4 );
    end

    if retCode ~= 0
        disp("Breaking main for loop.");
        break;  % safety
    end

    groupSyncWriteTxPacket(groupWriteNum);
    if getLastTxRxResult(port_num, params.PROTOCOL_VERSION) ~= COMM_SUCCESS
        disp("Error in sync write to servos");
    end
    % Clear syncwrite parameter storage
    groupSyncWriteClearParam(groupWriteNum);

    % fprintf("Overall time for one loop: %0.4f\n", (now-startLoopTime) * 24*60*60);

    curr_time = (now-start_time) * 24 * 60 * 60;
    curr_err = desiredPos - curr_pos; % Difference between sampled quintic position and measured position

    if isPlot
        err_vec(end+1,:) = curr_err;
        vel_vec(end+1,:) = curr_vel;
        command_vel_vec(end+1,:) = jointVel;
        desired_vel_vec(end+1,:) = desiredVelScaled;
        pos_vec(end+1,:) = curr_pos;
        time_vec(end+1) = curr_time;
    end
end

% Ensure that final speed is always 0
% for motor_idx=1:4
%     groupSyncWriteAddParam( groupWriteNum, params.DXL_LIST(i), 0, 4 );
% end
% groupSyncWriteTxPacket(groupWriteNum);
% groupSyncWriteClearParam(groupWriteNum);
write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_GOAL_VELOCITY, 0);
write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(2), params.ADDR_PRO_GOAL_VELOCITY, 0);
write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(3), params.ADDR_PRO_GOAL_VELOCITY, 0);
write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(4), params.ADDR_PRO_GOAL_VELOCITY, 0);

% Plot histories for each joint
if isPlot
    figure
    for joint=1:4
        subplot(4,3, 3*(joint-1)+1)
        plot(time_vec, err_vec(:,joint))
        title("Error history - Joint " + joint)
        ylim([-45 45])
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
end

end