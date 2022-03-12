function [x,y,z] = getLineCoord(pos1, pos2)
    % Returns elements of pos1 and pos2 separated by row
    x = [pos1(1),pos2(1)];
    y = [pos1(2),pos2(2)];
    z = [pos1(3),pos2(3)];
end