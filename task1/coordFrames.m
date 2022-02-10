close all
clear all
%% Parameters
frameLength = 10;
numJoints = 3;

alpha_1 = 50;

a_2 = 34;
alpha_2 = 90;


% Controlled angles
theta_2 = 0;
theta_3 = 0;


%% Transformations
% Transformation parameter syntax:
% struct("theta",   0, ... % Rotation along z axis
%         "alpha",    0, ... % Rotation of joint from z-axis
%         "a",        0, ... % Link length along x axis
%         "d",        0)); % Link length along z axis

% Transform list
T = zeros(4,4,numJoints+1);

% World frame
T(:,:,1) = eye(4); 
% Base of robot s.t. x axis pointing into link
T(:,:,2) = DHTransform(struct("theta",   theta_2, ... 
                            "alpha",    0, ... 
                            "a",        0, ...
                            "d",        34)); 
T(:,:,3) = DHTransform(struct("theta",   theta_3, ... 
                            "alpha",    -45, ... 
                            "a",        0, ... 
                            "d",        43)); 

%% Positions of joints
% joints = zeros(1,numJoints);

P_0 = T(:,:,1);
CoordX_0 = [1 0 0 frameLength;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
CoordY_0 = [1 0 0 0;
            0 1 0 frameLength;
            0 0 1 0;
            0 0 0 1];
CoordZ_0 = [1 0 0 0;
            0 1 0 0;
            0 0 1 frameLength;
            0 0 0 1];
joints(1) = struct("Pos", P_0, ...
                 "CoordX", CoordX_0, ...
                 "CoordY", CoordY_0, ...
                 "CoordZ", CoordZ_0);

% Apply transforms
for i=2:numJoints
    joints(i) = transformJoint(joints(i-1),T(:,:,i));
end

%% Plot joints

% Needed to display plot in 3d?
plot3(1,1,1)


% Draw global frame
drawFrame(joints(1))

% Draw all joints and connections
for i=2:numJoints
    % Draw line between previous joint and current joint
    drawLine(joints(i-1).Pos,joints(i).Pos)
    % Draw coordinate frame at joint
    drawFrame(joints(i))
end

% Set limits of build area
axis([-5,100,-5,100,-5,100])

grid on

