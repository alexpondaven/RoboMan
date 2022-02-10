function drawFrame(joint)
    % Extract struct members
    origin = joint.Pos;
    coordX = joint.CoordX;
    coordY = joint.CoordY;
    coordZ = joint.CoordZ;

    % Draw X coordinate frame vector from origin
    [x,y,z]= getLineCoord(origin(1:3,4),coordX(1:3,4));
    line('XData', x, 'YData', y,...
    'ZData', z, 'Color','r');
    % Draw Y coordinate frame vector from origin
    [x,y,z]= getLineCoord(origin(1:3,4),coordY(1:3,4));
    line('XData', x, 'YData', y,...
    'ZData', z, 'Color','g');
    % Draw Z coordinate frame vector from origin
    [x,y,z]= getLineCoord(origin(1:3,4),coordZ(1:3,4));
    line('XData', x, 'YData', y,...
    'ZData', z, 'Color','b');

end