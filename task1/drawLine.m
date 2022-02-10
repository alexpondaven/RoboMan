function drawLine(P_src, P_dst)
    [x,y,z] = getLineCoord(P_src(1:3,4),P_dst(1:3,4));
    line(x,y,z,'Color','k','LineWidth',5)
end