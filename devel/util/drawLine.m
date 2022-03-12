function drawLine(P_src, P_dst)
    % Draws line between two joints POSITIONS (x,y,z)
    [x,y,z] = getLineCoord(P_src,P_dst);
    line(x,y,z,'Color','k','LineWidth',5)
end