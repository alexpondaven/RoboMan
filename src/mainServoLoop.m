function retCode = mainServoLoop(coeffs, T, Tend, port_num, isPlot, vias)
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

% Initialise some variables
curr_pos = zeros(1,4);
curr_vel = zeros(1,4);
for i=1:4
    curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    curr_vel(i) = twos2decimal(read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY), 32);
end
curr_err = sampleQuintic(coeffs, T, 0) - curr_pos;    % difference from desired start point at T=0 and current point
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

% Store previous target segment
last_seg = false;   % If we are homing on the last segment

while ~( last_seg && all( abs(curr_err) < 10 ) )
    % TODO figure out what to do if control has not converged by the time ending
    if curr_time < Tend
        [desiredVel, targetIdx] = sampleQuinticVel(coeffs, T, curr_time);
        desiredPos = sampleQuintic(coeffs, T, curr_time);
    else
        % Aim to be at final position with zero velocity
        desiredPos = sampleQuintic(coeffs, T, Tend);
        desiredVel = zeros(1,4);
        targetIdx = size(T,1);
    end

    last_seg = targetIdx == size(T, 1);

    [jointVel, err_acc] = feedforwardPIcontrol(desiredVel, curr_err, err_acc);

    % Convert sampled joint velocity from ticks/s to rev/min. One unit = 0.229 RPM
    jointVel = round( (jointVel * 60 / 4096) / 0.229 );
    
    % startLoopTime = now;

    for i=1:4
        % Motor will clamp to velocity limits automatically
        % startReadTime = now;
        curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
        
        if curr_pos(i) > servoLimits(i,2) || curr_pos(i) < servoLimits(i,1)
            fprintf("VIOLATED JOINT LIMITS ON SERVO %d! EXITING\n", i);

            retCode = -1;
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_GOAL_VELOCITY, 0);
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(2), params.ADDR_PRO_GOAL_VELOCITY, 0);
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(3), params.ADDR_PRO_GOAL_VELOCITY, 0);
            write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(4), params.ADDR_PRO_GOAL_VELOCITY, 0);
            
            return % STOP EXECUTION
        end
        
        curr_vel(i) = twos2decimal(read4ByteTxRx (port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY), 32 );
        write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_GOAL_VELOCITY, typecast(int32(jointVel(i)), 'uint32') );
        
        % fprintf("Overall time for one read-write cycle: %0.4f\n", (now-startLoopTime) * 24*60*60);
    end

    % fprintf("Overall time for one loop: %0.4f\n", (now-startLoopTime) * 24*60*60);

    curr_time = (now-start_time) * 24 * 60 * 60;
    curr_err = desiredPos - curr_pos; % Difference between sampled quintic position and measured position

    if isPlot
        err_vec(end+1,:) = curr_err;
        vel_vec(end+1,:) = curr_vel;
        command_vel_vec(end+1,:) = jointVel;
        desired_vel_vec(end+1,:) = round( (desiredVel * 60 / 4096) / 0.229 );
        pos_vec(end+1,:) = curr_pos;
        time_vec(end+1) = curr_time;
    end
end

% Ensure that final speed is always 0
write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_GOAL_VELOCITY, 0);
write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(2), params.ADDR_PRO_GOAL_VELOCITY, 0);
write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(3), params.ADDR_PRO_GOAL_VELOCITY, 0);
write4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(4), params.ADDR_PRO_GOAL_VELOCITY, 0);
retCode = 0;

% Plot histories for each joint
if isPlot
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
end

end