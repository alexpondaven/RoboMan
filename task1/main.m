close all
clear all
% Array of controlled angles - ith angle is rotation of ith servo
% Last parameter is how open the gripper is
theta = [5,15,29,345,20];

%% Calculate theta for given x,y,z



%% Draw robot arm

% Move all angles
while 1
    for i=1:360
        coordFrames([i,i,i,i,20]);
    end
end

% Draw static default position
% coordFrames([0,0,0,0,10])