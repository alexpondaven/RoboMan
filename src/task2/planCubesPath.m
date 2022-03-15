function vias = planCubesPath(initCubesState, goalCubesState)
% planCubesPath     Determine via points to move cubes in initCubeState to
% desired goalCubeState
%
% Args
% initCubesState : array of initial cube states
%          [cube grid i, cube grid j, cube stack height, cube orientation]
%           cube orientation:
%           - `0`: upwards
%           - `90`: towards robot
%           - `-90`: away from robot
%           - `180`: downwards
% goalCubesState : array of final cube states
%
% Return
% Vias      : Via points between all waypoints

% Plan path algorithm
% - Pick up far away cubes with theta_g = 0
% - Otherwise theta_g = -pi/2
% - 

% cube_hold = [ [3,-8]; [5,-5]; [4, 0]; [9, 0]; [0, 4]; [6, 6] ].';


for i=1:size(initState,1)
    initCube = initCubesState(i,:)
    goalCube = goalCubesState(i,:)

    % Convert init and goal positions to x,y,z using grid pos and stack
    % height
    
    % Determine theta_g to grab cube (depending on cube holder)
    if initCube(1:2)==[3,-8] || initCube(1:2)==[9,0] || initCube(1:2)==[6,6]
        init_theta_g = 0;
    else
        init_theta_g = -pi/2;
    end

    % Determine if rotations are required

    % Determine where to translate cube
    
    % Get list of positions (x,y,z,theta_g) to go to
    positions = [];

    % Get via points between positions
    for pos=2:size(positions,2)-1
        grab()
        Asearch(pos, next_pos)
        drop()

    end

    % Translate



end

end