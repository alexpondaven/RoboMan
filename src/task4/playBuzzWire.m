%% Main function for Task 4

% Generate path
[path,angles] = genBuzzPath();
theta5 = arrayfun(buzzAngleToTheta5, angles);

% IK and interpolate