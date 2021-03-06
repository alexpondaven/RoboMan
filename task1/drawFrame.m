function drawFrame(origin, frameSize)
    % define length of coordinate frame vectors
%     frameSize = 20;

    pos = origin(1:3,4);
    % Draw X coordinate frame vector from origin
    [x,y,z]= getLineCoord(pos, pos + origin(1:3,1)*frameSize);
    line('XData', x, 'YData', y, 'ZData', z, 'Color','r');
    % Draw Y coordinate frame vector from origin
    [x,y,z]= getLineCoord(pos, pos + origin(1:3,2)*frameSize);
    line('XData', x, 'YData', y, 'ZData', z, 'Color','g');
    % Draw Z coordinate frame vector from origin
    [x,y,z]= getLineCoord(pos, pos + origin(1:3,3)*frameSize);
    line('XData', x, 'YData', y, 'ZData', z, 'Color','b');

end