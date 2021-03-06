
% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

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

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_PRO_DRIVE_MODE          = 10;

ADDR_PRO_GOAL_VELOCITY       = 104;
ADDR_PRO_PROFILE_VELOCITY    = 112;



%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_LIST = [11,12,13,14,15];
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM4';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ----- SET MOTION LIMITS ----------- %%
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;
%% ---------------------------------- %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

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
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

%% Initialize all dynamixels
for i=1:length(DXL_LIST)
    % Put actuator into Position Control Mode
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_OPERATING_MODE, 3);
%     % Set max position limit
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_MAX_POS, MAX_POS);
%     % Set min position limit
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_MIN_POS, MIN_POS);
    % Set Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

    % Set profile velocity - smoother motion
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PROFILE_VELOCITY, 1024);
    % Profile acceleration
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), 108, );

    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    else
        fprintf('Dynamixel %d has been successfully connected \n', i);
    end

end

%% --- Dynamixel Setup End --- %%

%% Code snippet for Task 1-1 (Read Encoders)
% i = 0;
% j = 0;
% while (j<400)
%     j = j+1;
%     pos_arr = [0 0 0 0 0];
%     
%     for i = 1:length(DXL_LIST)
%         % Read present position
%         pos_arr(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PRESENT_POSITION);
%         dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
% 
%         % error handling
%         dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
% 
%         if dxl_comm_result ~= COMM_SUCCESS
%             fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%         elseif dxl_error ~= 0
%             fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%         end
%         if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
%             break;
%         end
%         %end error handling
% 
%         fprintf('[ID:%03d] Pos: %03d\t', DXL_LIST(i), typecast(uint32(pos_arr(i)), 'int32'));
%     end
%     fprintf('\n');
% end

%% Code Snippet for Task 1-3 (Seq of pos commands)
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(4), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(4), ADDR_PRO_GOAL_POSITION, 2048);  % 180
% pause(1);
% 
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(3), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(3), ADDR_PRO_GOAL_POSITION, 1024);  % 180
% pause(1);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(3), ADDR_PRO_GOAL_POSITION, 2048);  % 180
% pause(1);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(3), ADDR_PRO_GOAL_POSITION, 3072);  % 180
% pause(1);
% 
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(3), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(4), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

%% Code snippet for inverse kinematic control

%%% Nodding function by varying theta_G
% theta_G = 0;
% for theta_G = -pi/4:0.1:pi/4
%     theta = inverseKinDynamixel(200,0,150,theta_G,10)
%     for i=1:length(DXL_LIST)
%         write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
%     end
%     pause(0.1)
% end

%%% Rotate box starting from top
% thetaL = [];
% %prepare
% thetaL(end+1,:) = inverseKinDynamixel(150,0,65,-pi/2,deg2rad(95))
% % open at position
% thetaL(end+1,:) = inverseKinDynamixel(225,0,65,-pi/2,deg2rad(95))
% 
% %closed on cube
% thetaL(end+1,:) = inverseKinDynamixel(225,0,65,-pi/2,deg2rad(212))
% 
% % raise cube - tends to hit everything
% thetaL(end+1,:) = inverseKinDynamixel(210,0,100,-pi/4,deg2rad(212))
% 
% % rotate cube
% thetaL(end+1,:) = inverseKinDynamixel(210,0,100,0,deg2rad(212))
% 
% % place back
% thetaL(end+1,:) = inverseKinDynamixel(225,0,60,0,deg2rad(212))
% 
% % Drop
% thetaL(end+1,:) = inverseKinDynamixel(225,0,60,0,deg2rad(200))

%% Code snippet for box manipulation

% % Rotate box grabbing from side
% thetaL = [];
% %prepare
% thetaL(end+1,:) = inverseKinDynamixel(20,0,200,0,deg2rad(95))
% % open at position
% thetaL(end+1,:) = inverseKinDynamixel(225,0,45,0,deg2rad(95))
% 
% %closed on cube
% thetaL(end+1,:) = inverseKinDynamixel(225,0,45,0,deg2rad(212))
% 
% % raise cube - tends to hit everything
% thetaL(end+1,:) = inverseKinDynamixel(210,0,100,0,deg2rad(212))
% 
% % go to 3
% thetaL(end+1,:) = inverseKinDynamixel(155,155,100,-pi/2,deg2rad(212))
% 
% % lower
% thetaL(end+1,:) = inverseKinDynamixel(155,155,65,-pi/2,deg2rad(212))
% 
% % drop it
% thetaL(end+1,:) = inverseKinDynamixel(155,155,65,-pi/2,deg2rad(95))
% 
% %prepare
% thetaL(end+1,:) = inverseKinDynamixel(20,0,200,-pi/2,deg2rad(95))
% 
% POSITIONS OF ALL CUBES
% 1) grids=(3,-8) - from side
%   (75,-200,40,0,deg2rad(212))
% 2) grids=(9,0) -from side
%   (225,0,40,0,deg2rad(212))
% 3) grids=(6,6) - from side
%   (150,150,40,0,deg2rad(212))
% 4) grids=(5,-5) - from top
%   (125,-125,50,-pi/2,deg2rad(212))
% 5) grids=(4,0) - from top
%   (100,0,50,-pi/2,deg2rad(212))
% 6) grids = (0,4) - from top
%   (0,100,50,-pi/2,deg2rad(212))

% for t=1:size(thetaL,1)
%     for i=1:length(DXL_LIST)
%         write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, thetaL(t,i));
%     end
%     pause(2)
% end


%% Code snippet for Draw square
% theta = inverseKinDynamixel(200,-100,50,theta_G,10)
% for i=1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
% end
% pause(3);
% theta = inverseKinDynamixel(100,-100,50,theta_G,10)
% for i=1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
% end
% pause(3);
% theta = inverseKinDynamixel(100,100,50,theta_G,10)
% for i=1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
% end
% pause(3);
% theta = inverseKinDynamixel(200,100,50,theta_G,10)
% for i=1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
% end
% pause(3);

%% Code snippet for Line path sampling
z=100;
% Corners
corners = [175 -100 z
            175 100 z
            225 100 z
            225 -100 z];
        
corners = [ 125 190 z
            100 200 z
];

"TIME BASED"
        
for j=1:size(corners, 1)
    theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), 0, deg2rad(95))
    for i=1:length(DXL_LIST)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
    end
    pause(2);
end

% "SAMPLING BASED"
% 
% thetaList = [];
% 
% posPath = genLineTraj(corners(1,:),corners(2,:));
% 
% for i=1:size(posPath,1)
%     pos = posPath(i,:);
%     theta = inverseKinDynamixel(pos(1),pos(2),pos(3),0,deg2rad(95));
% %         coordFrames(theta);
%     thetaList(end+1, :) = theta;
% end
% 
% posPath = genLineTraj(corners(2,:),corners(3,:));
% 
% for i=1:size(posPath,1)
%     pos = posPath(i,:);
%     theta = inverseKinDynamixel(pos(1),pos(2),pos(3),0,deg2rad(95));
% %         coordFrames(theta);
%     thetaList(end+1, :) = theta;
% end
% 
% posPath = genLineTraj(corners(3,:),corners(4,:));
% 
% for i=1:size(posPath,1)
%     pos = posPath(i,:);
%     theta = inverseKinDynamixel(pos(1),pos(2),pos(3),0,deg2rad(95));
% %         coordFrames(theta);
%     thetaList(end+1, :) = theta;
% end
% 
% posPath = genLineTraj(corners(4,:),corners(1,:));
% 
% for i=1:size(posPath,1)
%     pos = posPath(i,:);
%     theta = inverseKinDynamixel(pos(1),pos(2),pos(3),0,deg2rad(95));
% %         coordFrames(theta);
%     thetaList(end+1, :) = theta;
% end

% for iteration=1:2
%     for t=1:size(thetaList,1)
%         for i=1:length(DXL_LIST)
% %             thetaList(t, i)
%             write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, thetaList(t,i));
%         end
% %         pause(0.05)
%     end
% end

%% -- Dynamixel Cleanup Start -- %%
for i=1:length(DXL_LIST)
    % Disable Dynamixel Torque
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

% close all;
clear all;