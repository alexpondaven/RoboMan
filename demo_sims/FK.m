% Increment all angles
theta = inverseKin(200,-300,50,-pi/6,10);
while 1
    for i=0:0.01:1
        coordFrames([theta(1)+i*pi/2, theta(2)+i*pi/3, theta(3)+i*pi/6, theta(4)+i*pi/4]);
        drawnow
    end
end