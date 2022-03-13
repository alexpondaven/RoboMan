% t1=1;
% t2=1;
% b3=2;
% b2=3;
% b1=0;
% 
% % coeffs = [a12,a13,a21,a22,a23]
% 
% A=[t1^2 t1^3 0 0 0
%    2*t1 3*t1^2 -1 0 0
%    0 0 t2 t2^2 t2^3
%    0 0 1 2*t2 3*t2^2
%    2 6*t1 0 -2 0];
% 
% b = [b2-b1
%      0
%      b3-b2
%      0
%      0];
% 
% linsolve(A,b)

% vias = [0 0 0 0
%         1 4 1 1
%         2 2 2 2];

%% Testing for toy case
vias = [0 0 0 0
        1 4 9 1
        2 5 2 2
        8 2 4 6];

%% Testing for square case
z=90;
% Corners
square = [ 175 -50 z
           175  50 z
           75   50 z
           75  -50 z ];

square(5,:) = square(1,:);  % make it complete the square

% interpolate lines between corners
corners = [];
numPoints = 10;
for i=2:length(square)
    corners = [corners; linearInterpolate(square(i-1,:), square(i,:), numPoints) ];
end

vias = [];
for j=1:size(corners, 1)
    % corners(j,:)
    theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2, GRIP_POS);
    GRIP_ANGLE = theta(5);
    vias = [vias; theta(1:4)];
end
Tend = 15;

vias % For debugging

% vias = [[0:10]' [0:10]' [0:10]' [0:10]'];
%% Cubic Interpolation
% [coeffs, T] = interpCubicTraj(vias,Tend)
% 
% plotCubicInterp(vias, coeffs, T)

%% Quintic Interpolation
T = assignViaTimes(vias,Tend)
coeffs = interpQuinticTraj(vias, T)

plotQuinticInterp(vias, coeffs, T)