% vias = [0 0 0 0
%         1 4 9 1
%         2 0 2 2
%         3 4 4 6
%         4 0 4 6
%         5 2 4 6
%         6 2 4 6
%         7 2 4 6];
% % Tend =15;

% [T, Tend] = assignViaTimes(vias, 'acc');    % Tend no longer used
% coeffs = interpQuinticTraj(vias, T);
% plotQuinticInterp(vias, coeffs, T)

% We test on a real-life example

servoLimits = getServoLimits();
velocityLimit = getDXLSettings().velocityLimit;
viaTimeInterpMethod = getDXLParams().viaTimeInterpMethod;

%% Path Planning
% Measure curr pos for robustness

%% moving to the pen
z_pick = 60;
z_prep = z_pick + 50;
coords = [
    120 120 90 0             % Current position (sample)
    125 -125 z_pick 0        % Intermediate position to grab the pen
    150 -150 z_pick 0        % Pen grabbing
];
% Interpolate between moving through via points
waypoints_pick = [];
num_points_pick = 20;
for i=2:size(coords,1)
    waypoints_pick = [waypoints_pick; linearInterpolate(coords(i-1,:), coords(i,:), num_points_pick) ];
end

% Calculate IK for picking up marker
vias_pick = [];
for j=1:size(waypoints_pick, 1)
    theta = inverseKinDynamixel2(waypoints_pick(j, 1), waypoints_pick(j, 2), waypoints_pick(j, 3), waypoints_pick(j, 4), true);
    vias_pick = [vias_pick; theta(1:4)];
end

figure

% Assign times for picking up marker
[T_pick, coeffs_pick, Tend_pick] = assignViaTimes2(vias_pick, viaTimeInterpMethod);    % Tend no longer used
plotQuinticInterp(vias_pick, coeffs_pick, T_pick);

[T, Tend] = assignViaTimes(vias_pick, 'acc');
coeffs = interpQuinticTraj(vias_pick, T);
plotQuinticInterp(vias_pick, coeffs, T);

[T2, Tend2] = assignViaTimes(vias_pick, 'acc2');
coeffs2 = interpQuinticTraj(vias_pick, T2);
plotQuinticInterp(vias_pick, coeffs2, T2);

fprintf("Tend2: %0.4f, Tend (baseline): %0.4f\n", Tend_pick, Tend);