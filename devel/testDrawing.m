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
  [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', ...
  'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
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
    viaTimeInterpMethod = getDXLParams().viaTimeInterpMethod;
    
    %% Path Planning
    % Measure curr pos for robustness
    curr_pos = zeros(1,4);
    for i=1:4
        curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    end
    currEndpointCoords = getCurrEndpointCoords(curr_pos);
    
    %% moving to the pen
    z_pick = 60;
    z_prep = z_pick + 50;
    coords = [
        currEndpointCoords          % Current position
        125 -125 z_pick 0        % Intermediate position to grab the pen
        150 -150 z_pick 0        % Pen grabbing
    ];
    % Interpolate between moving through via points
    waypoints_pick = [];
    num_points_pick = 20;
    for i=2:size(coords,1)
        waypoints_pick = [waypoints_pick; linearInterpolate(coords(i-1,:), coords(i,:), num_points_pick) ];
    end

    % Calculate IK for picking up marker
    vias_pick = [];
    for j=1:size(waypoints_pick, 1)
        theta = inverseKinDynamixel2(waypoints_pick(j, 1), waypoints_pick(j, 2), waypoints_pick(j, 3), waypoints_pick(j, 4), true);
        vias_pick = [vias_pick; theta(1:4)];
    end

    % Assign times for picking up marker
    [T_pick, Tend_pick] = assignViaTimes(vias_pick, viaTimeInterpMethod);    % Tend no longer used
    coeffs_pick = interpQuinticTraj(vias_pick, T_pick);
    figure
    plotQuinticInterp(vias_pick, coeffs_pick, T_pick);

    % close gripper here

    %% Drawing
    coords = [
        150 -150 z_pick 0        % Pen grabbing
        150 -150 z_prep 0        % Pen lifting
        ];
    waypoints_draw = [];
    for i=2:size(coords,1)
        waypoints_draw = [waypoints_draw; linearInterpolate(coords(i-1,:), coords(i,:), num_points_pick) ];
    end

    %% Drawing lines
    z = 50;                         % How low to place the thing
    % lines coords
    % TODO make this into an argument?
%     coords = [  200 60  z_prep 0
%                 200 60  z      0
%                 200 140 z      0
%                 125 140 z      0
%                 200 60  z      0 ];
    
    % DEMO DAY
    coords = [  175 100  z_prep 0
                175 100  z      0
                175 200 z      0
                125 150 z      0
                175 150  z      0 ];
    
    % Interpolate between all points so far
    num_points_lines = 50;
    for i=2:size(coords,1)
        waypoints_draw = [waypoints_draw; linearInterpolate(coords(i-1,:), coords(i,:), num_points_lines) ];
    end

    %% Drawing semicircle
%     origin=[200,100,z];
%     radius=40;
%     num_points_circle=100;
%     
%     % generate way points for the circle. 
%     % TODO make into an argument
%     circle = semicircle3(origin,radius,num_points_circle);
%     waypoints_draw = [waypoints_draw; circle];

    % DEMO DAY
    startP=[175,150,z];
    center = [200,150,z];
    circle_theta=3*pi/2; % 270 degrees
    num_points_circle=100;
    
    % generate way points for the circle. 
    % TODO make into an argument
    circle = semicircle6(startP, center, circle_theta, num_points_circle);
    waypoints_draw = [waypoints_draw; circle];



    % Calculate IK for drawing
    vias_draw = [];
    for j=1:size(waypoints_draw, 1)
        theta = inverseKinDynamixel2(waypoints_draw(j, 1), waypoints_draw(j, 2), waypoints_draw(j, 3), waypoints_draw(j, 4), true);
        vias_draw = [vias_draw; theta(1:4)];
    end
    
    % Assign times for drawing
    [T_draw, Tend_draw] = assignViaTimes(vias_draw, viaTimeInterpMethod);    % Tend no longer used
    coeffs_draw = interpQuinticTraj(vias_draw, T_draw);
    figure
    plotQuinticInterp(vias_draw, coeffs_draw, T_draw);

    waypoints_move = [];
    % Generate for returning to drop off
    coords = [  waypoints_draw(end, :)
                waypoints_draw(end, :) + [0 0 50 0]     % Raise the gripper
                125 -125 z_prep 0               % Back to hovering over holder
                150 -150 z_prep 0               % Back to hovering over holder
                150 -150 z_pick 0               % move down to drop marker
    ];
    for i=2:size(coords,1)
        waypoints_move = [waypoints_move; linearInterpolate(coords(i-1,:), coords(i,:), num_points_pick) ];
    end

    % Calculate IK for drawing
    vias_move = [];
    for j=1:size(waypoints_move, 1)
        theta = inverseKinDynamixel2(waypoints_move(j, 1), waypoints_move(j, 2), waypoints_move(j, 3), waypoints_move(j, 4), true);
        vias_move = [vias_move; theta(1:4)];
    end
    
    % Assign times for drawing
    [T_move, Tend_move] = assignViaTimes(vias_move, viaTimeInterpMethod);    % Tend no longer used
    coeffs_move = interpQuinticTraj(vias_move, T_move);
    figure
    plotQuinticInterp(vias_move, coeffs_move, T_move);

    % open gripper here

    %% Placing
    coords = [ 150 -150 z_pick 0
               150 -150 z_prep 0
               125 -125 z_prep 0
    ];
    waypoints_drop = [];
    for i=2:size(coords,1)
        waypoints_drop = [waypoints_drop; linearInterpolate(coords(i-1,:), coords(i,:), num_points_pick) ];
    end

    % Calculate IK for picking up marker
    vias_drop = [];
    for j=1:size(waypoints_drop, 1)
        theta = inverseKinDynamixel2(waypoints_drop(j, 1), waypoints_drop(j, 2), waypoints_drop(j, 3), waypoints_drop(j, 4), true);
        vias_drop = [vias_drop; theta(1:4)];
    end

    % Assign times for picking up marker
    [T_drop, Tend_drop] = assignViaTimes(vias_drop, viaTimeInterpMethod);    % Tend no longer used
    coeffs_drop = interpQuinticTraj(vias_drop, T_drop);
    figure
    plotQuinticInterp(vias_drop, coeffs_drop, T_drop);

    % end path planning

    %% Main movement
    setGripperPos(true, port_num);

    mainServoLoop2(coeffs_pick, T_pick, Tend_pick, port_num, true, vias_pick);

    setGripperPos(false, port_num);     % Pick up marker

    mainServoLoop2(coeffs_draw, T_draw, Tend_draw, port_num, true, vias_draw);
    mainServoLoop2(coeffs_move, T_move, Tend_move, port_num, true, vias_move);  % make it stop before picking up

    setGripperPos(true, port_num);      % Drop marker

    mainServoLoop2(coeffs_drop, T_drop, Tend_drop, port_num, true, vias_drop);

    % %% Lift up pen by <variable>
    % pick = [pick(end,:); pick(end,:) + [0 0 50 0]];

    % corners = [];
    % numPoints = 10;
    % for i=2:size(pick,1)
    %     corners = [corners; linearInterpolate(pick(i-1,:), pick(i,:), numPoints) ];
    % end

    % curr_pos = zeros(1,4);  % Measure curr pos for robustness
    % for i=1:4
    %   curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    % end
    % % currEndpointCoords = getCurrEndpointCoords(curr_pos);
    % %  calculate vias with adajustment to the gripper angle_thetaG  
    % vias = curr_pos;
    % for j=1:size(corners, 1)
    %     % corners(j,:)
    %     % theta = inverseKinDynamixel3(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2,pi/4,GRIP_POS);
    %     theta = inverseKinDynamixel2(corners(j, 1), corners(j, 2), corners(j, 3), 0, true);
    %     vias = [vias; theta(1:4)];
    % end

    % % Interpolate between waypoints
    % [T, Tend] = assignViaTimes(vias, viaTimeInterpMethod);    % Tend no longer used
    % coeffs = interpQuinticTraj(vias, T);
    % figure
    % plotQuinticInterp(vias, coeffs, T);

    % mainServoLoop2(coeffs, T, Tend, port_num, true, vias);

    % %% End lifting pen
      
    % % Draw
    % % Z-axis offset
    % z = 47;
    % z_prepare = 80;
    % % lines coords
    % coords = [ 200 60  z_prepare
    %             200 60  z
    %             200 140 z
    %             125 140 z
    %             200 60  z ];

    % % interpolate lines between corners
    % corners = [];
    % numPoints = 50;
    % for i=2:size(coords,1)
    %     corners = [corners; linearInterpolate(coords(i-1,:), coords(i,:), numPoints) ];
    % end
    
    % curr_pos = zeros(1,4);
    % for i=1:4
    %     curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    % end
    
    % currEndpointCoords = getCurrEndpointCoords(curr_pos);
    % %  calculate vias with adajustment to the gripper angle_thetaG  
    % vias = curr_pos;
    % for j=1:size(corners, 1)
    %     % corners(j,:)
    %     %  theta = inverseKinDynamixel3(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2,pi/4,GRIP_POS);
    %     theta = inverseKinDynamixel2(corners(j, 1), corners(j, 2), corners(j, 3), 0, true);
    %     vias = [vias; theta(1:4)];
    % end

    % % Interpolate between waypoints
    % [T, Tend] = assignViaTimes(vias, 'dvel');    % Tend no longer used
    % coeffs = interpQuinticTraj(vias, T);
    % figure
    % plotQuinticInterp(vias, coeffs, T);
    
    % mainServoLoop2(coeffs, T, Tend, port_num, true, vias);
    
    % %Parms of the circle
    % origin=[200,100,z];
    % radius=40;
    % num_points=50;
    
    % % generate way points for the circle
    % circle=semicircle3(origin,radius,num_points);
    
    
    % curr_pos = zeros(1,4);
    % for i=1:4
    % curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    % end
    
    % currEndpointCoords = getCurrEndpointCoords(curr_pos);
    % %  calculate vias with adajustment to the gripper angle_thetaG  
    % vias = curr_pos;
    % for j=1:size(circle, 1)
    % theta = inverseKinDynamixel2(circle(j, 1), circle(j, 2), circle(j, 3), 0, true);
    % vias = [vias; theta(1:4)];
    % end
    
    % % Interpolate between waypoints
    % [T, Tend] = assignViaTimes(vias, 'dvel');    % Tend no longer used
    % coeffs = interpQuinticTraj(vias, T);
    % figure
    % plotQuinticInterp(vias, coeffs, T);
    
    % mainServoLoop2(coeffs, T, Tend, port_num, true, vias);
            

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


