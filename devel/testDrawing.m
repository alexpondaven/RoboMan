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
    z=40;
    coords = [ 200 60  z
               200 140 z
               125 140 z
               200 60  z ];

     % interpolate lines between corners
     corners = [];
     numPoints = 10;
     for i=2:length(coords)
        corners = [corners; linearInterpolate(coords(i-1,:), coords(i,:), numPoints) ];
     end

     GRIP_POS = deg2rad(232);

     curr_pos = zeros(1,4);
     for i=1:4
         curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
     end
     
     currEndpointCoords = getCurrEndpointCoords(curr_pos);
     %  calculate vias with adajustment to the gripper angle_thetaG  
     vias = curr_pos;
     for j=1:size(corners, 1)
         % corners(j,:)
         %  theta = inverseKinDynamixel3(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2,pi/4,GRIP_POS);
         theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), 0, GRIP_POS);
         vias = [vias; theta(1:4)];
     end

      % Interpolate between waypoints
    [T, Tend] = assignViaTimes(vias, 'dvel');    % Tend no longer used
    coeffs = interpQuinticTraj(vias, T);
    figure
    plotQuinticInterp(vias, coeffs, T);

    mainServoLoop(coeffs, T, Tend, port_num, true, vias);

    %Parms of the circle
    origin=[200,100,40];
    radius=40;
    num_points=20;

    circle=semicircle2(origin,radius,num_points);


    curr_pos = zeros(1,4);
     for i=1:4
         curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
     end
     
     currEndpointCoords = getCurrEndpointCoords(curr_pos);
     %  calculate vias with adajustment to the gripper angle_thetaG  
     vias = curr_pos;
     for j=1:size(circle, 1)
         theta = inverseKinDynamixel(circle(j, 1), circle(j, 2), circle(j, 3), 0, GRIP_POS);
         vias = [vias; theta(1:4)];
     end

      % Interpolate between waypoints
    [T, Tend] = assignViaTimes(vias, 'dvel');    % Tend no longer used
    coeffs = interpQuinticTraj(vias, T);
    figure
    plotQuinticInterp(vias, coeffs, T);

    mainServoLoop(coeffs, T, Tend, port_num, true, vias);

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