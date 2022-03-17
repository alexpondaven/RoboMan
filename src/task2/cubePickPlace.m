function retCode = cubePickPlace(goalPos, cubePos, nextStartPos, pickOrPlace, port_num)
% cubePickPlace is a high-level function that handles:
% picking / placing cubes from given A* waypoints.
% 1. *entry point: End effector at goal point
% 2. Move end effector to grasp cube (hard-code downwards 20mm?)
% 3. Close Gripper
% 4. Move end effector to next start point (hard-code upwards 20mm?)
% 5. *exit point: End effector follows the next set of via points
%
% Args:
% goalPos      : (x,y,z,theta_g) of end effector. The goal position of the last A* path.
% cubePos      : (x,y,z,theta_g) of end effector. The orientation required to grasp the cube.
% nextStartPos : (x,y,z,theta_g) of end effector. The first position of the next A* path.
% pickOrPlace  : (bool) true: Robot picks up cube. (assume gripper is open at first)
%                      false: Robot deposits cube. (assumes gripper is closed at first)
% port_num     : handler number from earlier.
%
% Returns:
% retCode      : 0 on success

    %% FUNCTION SPECIFIC TUNE VALUES
    % Tend = 3;       % Conservative value, we really don't want to screw this up
    numPoints = 20;  % Again, conservative
    startGripperPos = pickOrPlace;
    endGripperPos = ~pickOrPlace;
    params = getDXLParams();

    disp("[cubePickPlace] asserting initial gripper pos");

    % assert that gripper is indeed in desired position
    if pickOrPlace == true 
        setGripperPos(true, port_num);  % open gripper
    else
        setGripperPos(false, port_num); % make sure grip is tight
    end

    disp("[cubePickPlace] Moving from goalPos -> cubePos");

    %% Generate trajectory from goalPos -> cubePos
    viaCoords = linearInterpolate(goalPos, cubePos, numPoints);
    viaAngles = zeros( size(viaCoords) );
    for i=1:size(viaCoords, 1)
        inverseKinRes = inverseKinDynamixel2( viaCoords(i,1), viaCoords(i,2), viaCoords(i,3), viaCoords(i,4), startGripperPos);
        viaAngles(i,:) = inverseKinRes(1:4);
    end
    
    % To be robust we take current position into account
    curr_pos = zeros(1,4);
    curr_pos(1) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_PRESENT_POSITION);
    curr_pos(2) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(2), params.ADDR_PRO_PRESENT_POSITION);
    curr_pos(3) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(3), params.ADDR_PRO_PRESENT_POSITION);
    curr_pos(4) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(4), params.ADDR_PRO_PRESENT_POSITION);
    viaAngles = [curr_pos; viaAngles];

    % We use the primitive function because we will only have 1 via to pass through
    [T, Tend] = assignViaTimes(viaAngles, 'dvel');
    coeffs = interpQuinticTraj(viaAngles, T);
    Tend
    % [coeffs, T, Tend] = interpViaPoints(viaAngles, true);
    if mainServoLoop(coeffs, T, Tend, port_num, true, viaAngles) ~= 0
        disp("Error in [cubePickPlace] goalPos -> cubePos");
        retCode = -1;
        return;
    end

    disp("[cubePickPlace] asserting second gripper pos");

    %% open/let go of gripper
    if pickOrPlace == true 
        setGripperPos(false, port_num); % Grip cube
    else
        setGripperPos(true, port_num);  % let cube go
    end

    disp("[cubePickPlace] Moving from cubePos -> nextStartPos");
    
    %% Generate trajectory from cubePos -> nextStartPos
    viaCoords = linearInterpolate(cubePos, nextStartPos, numPoints);
    viaAngles = zeros( size(viaCoords) );
    for i=1:size(viaCoords, 1)
        inverseKinRes = inverseKinDynamixel2( viaCoords(i,1), viaCoords(i,2), viaCoords(i,3), viaCoords(i,4), endGripperPos);
        viaAngles(i,:) = inverseKinRes(1:4);
    end
    
    % To be robust we take current position into account
    curr_pos = zeros(1,4);
    curr_pos(1) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(1), params.ADDR_PRO_PRESENT_POSITION);
    curr_pos(2) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(2), params.ADDR_PRO_PRESENT_POSITION);
    curr_pos(3) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(3), params.ADDR_PRO_PRESENT_POSITION);
    curr_pos(4) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(4), params.ADDR_PRO_PRESENT_POSITION);
    viaAngles = [curr_pos; viaAngles];
    
    % We use the primitive function because we will only have 1 via to pass through
    [T, Tend] = assignViaTimes(viaAngles, 'dvel');
    coeffs = interpQuinticTraj(viaAngles, T);
    Tend
    % [coeffs, T, Tend] = interpViaPoints(viaAngles, true);
    if mainServoLoop(coeffs, T, Tend, port_num, true, viaAngles) ~= 0
        disp("Error in [cubePickPlace] cubePos -> nextStartPos");
        retCode = -1;
        return;
    end

    retCode = 0;
end