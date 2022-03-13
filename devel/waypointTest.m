%% Code to test cubic interpolation

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

index = 1;
dxl_comm_result = params.COMM_TX_FAIL;           % Communication result
dxl_goal_position = [params.DXL_MINIMUM_POSITION_VALUE params.DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


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

initDynamixels(port_num);

%% Code snippet for testing
z=90;
% Corners


square = [ 175 -50 z
           175  50 z
           75   50 z
           75  -50 z ];

square(5,:) = square(1,:);  % make it complete the square

% interpolate lines between corners
corners = []
numPoints = 10;
for i=2:length(square)
    corners = [corners; linearInterpolate(square(i-1,:), square(i,:), numPoints) ];
end

GRIP_POS = deg2rad(232);
GRIP_ANGLE = 0;
        
%% ~ Code snippet for testing with standard inverse kinematics
% for j=1:length(corners)
%     theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2, GRIP_POS);
%     for i=1:length(DXL_LIST)
%         write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
%         % curr_servo_theta(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PRESENT_POSITION);
%     end 
%     pause(5);
% end

%% ~ Code snippet for testing with waypoint interpolation
% Get thetas for corners[coeff]
vias = [];
for j=1:size(corners, 1)
    % corners(j,:)
    theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2, GRIP_POS);
    GRIP_ANGLE = theta(5);
    vias = [vias; theta(1:4)];
end
Tend = 15;

% Interpolate between waypoints
[coeffs, T] = interpQuinticTraj(vias, Tend);

plotQuinticInterp(vias, coeffs, T)

disp("TIME BASED");
start = now;

disp("Goal Theta");
init_theta = [vias(1,:) GRIP_ANGLE]
goal_theta = [vias(end,:) GRIP_ANGLE]
vel_hist = [];

% Change to position control mode and get to start position
% setServoMode('pos', port_num);
% writeToServos(init_theta, 'pos', port_num);

% want to ensure we don't end up with a deadzone
% while(~all( abs(curr_servo_theta(1:4)-goal_theta(1:4)) < 50  ))

% We want initial values
curr_servo_theta = zeros(1,5);
for i=1:length(params.DXL_LIST)
    curr_servo_theta(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_GOAL_POSITION);
end

% Change to velocity control mode and get to end position
setServoMode('pos', port_num);
% setServoMode('vel', port_num);
while 1
    curTime = (now - start) * 24*60*60; % Fractional part of `now` is 24hrs
                                        % we want to get current time in seconds
    if curTime<(Tend)
        % ~ IF ATTEMPTING POS MODE
        theta = [sampleQuintic(coeffs, T, curTime*1.1) GRIP_ANGLE];
        % theta = cast([sampleQuinticVel(coeffs, T, curTime) 0], 'uint32')
        
        % ~ IF ATTEMPTING VELOCITY MODE
        % theta = sampleQuintic(coeffs, T, curTime);
        % desiredTraj = curr_servo_theta(1:4) - theta; % this is essentially P control
        % desiredTraj = [cast(desiredTraj, 'uint32') 0]

    else
        % Clip to final theta position
        theta = [vias(end,:) GRIP_ANGLE];
        % theta = [0 0 0 0 0];

        % desiredTraj = [0 0 0 0 0];
    end

    % fprintf("Commanded Pos at t:%0.2f | t_1: %0.1f | t_2: %0.1f | t_3: %0.1f | t_4: %0.1f | t_5: %0.1f\n", ...
    %     curTime, theta(1), theta(2), theta(3), theta(4), theta(5)    );
    % fprintf("Current Position         | t_1: %0.1f | t_2: %0.1f | t_3: %0.1f | t_4: %0.1f | t_5: %0.1f\n\n", ...
    %     curr_servo_theta(1), curr_servo_theta(2), curr_servo_theta(3), ...
    %     curr_servo_theta(4), curr_servo_theta(5));
    
    % writeToServos(desiredTraj, 'vel', port_num);
    writeToServos(theta, 'pos', port_num);

    joint_vel = zeros(1,5);
    for i=1:length(params.DXL_LIST)
        joint_vel(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY);
        % curr_servo_theta(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_GOAL_POSITION);
    end
    
    vel_hist = [vel_hist; joint_vel];
    
    if curTime >= Tend
        break
    end
end

% Change to position control mode and get to end position
% setServoMode('pos', port_num);
% writeToServos(goal_theta, 'pos', port_num);


vel_hist


%% -- Dynamixel Cleanup Start -- %%
for i=1:length(params.DXL_LIST)
    % Disable Dynamixel Torque
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
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

% close all;
clear