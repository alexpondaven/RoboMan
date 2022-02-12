function drawGripper(origin, gripperSize)
    % Draw gripper:
    % tl       tr
    % |        |
    % |        |
    % |________|
    % bl  |    br
    %     |
    % <-->
    % gripperSize

    % Parameters:
    % - origin defines location to place gripper
    % - gripperSize is how open the gripper is
    
    % Coordinates
    pos = origin(1:3,4);
    bl = pos - origin(1:3,3)*gripperSize; % Bottom left
    br = pos + origin(1:3,3)*gripperSize; % Bottom right
    tl = bl + origin(1:3,1)*gripperSize; % Top left
    tr = br + origin(1:3,1)*gripperSize; % Top right

    % Draw base of gripper
    % Bottom right frame
    drawLine(pos, br)
    % Bottom left frame
    drawLine(pos, bl)
    % Left frame
    drawLine(bl, tl)
    % Right frame
    drawLine(br, tr)
    
end