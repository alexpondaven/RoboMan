close all
clear all
%% Parameters
frameLength = 10;
% numJoints = 3;

alpha_1 = 50;

a_2 = 34;
alpha_2 = 90;


% Controlled angles
theta_1 = 5; % Rotation of base of first servo
theta_2 = 15; % Rotation of base of second servo
theta_3 = 29; % Rotation of third servo
theta_4 = 345; % Rotation of fourth servo



%% Transformations

% Transform list
% T = zeros(4,4,numJoints);

% World frame
T(:,:,1) = eye(4); 
% Base of robot s.t. x axis pointing into link
T(:,:,2) = DHTransform(struct("theta",   theta_1, ... % Base of first servo
                            "alpha",    0, ... 
                            "a",        0, ...
                            "d",        34));
T(:,:,3) = DHTransform(struct("theta",   90, ... % Intermediate
                            "alpha",    90, ... 
                            "a",        0, ... 
                            "d",        0)); 
T(:,:,4) = DHTransform(struct("theta",   theta_2, ... % Base of second servo
                            "alpha",    0, ... 
                            "a",        43, ... 
                            "d",        0)); 
T(:,:,5) = DHTransform(struct("theta",   -90, ... % Elbow between servos
                            "alpha",    0, ... 
                            "a",        128, ... 
                            "d",        0)); 
T(:,:,6) = DHTransform(struct("theta",   theta_3, ...  % Base of third servo
                            "alpha",    0, ... 
                            "a",        24, ... 
                            "d",        0));
T(:,:,7) = DHTransform(struct("theta",   theta_4, ... % Base of fourth servo
                            "alpha",    0, ... 
                            "a",        124, ... 
                            "d",        0)); 
T(:,:,8) = DHTransform(struct("theta",   0, ...% End effector
                            "alpha",    0, ... 
                            "a",        126, ... 
                            "d",        0)); 

%% Positions of joints
% joints = zeros(1,numJoints);

joints(:,:,1) = T(:,:,1);

% Apply transforms
for i=2:size(T,3)
    joints(:,:,i) = joints(:,:,i-1) * T(:,:,i);
end

%% Plot joints

% Needed to display plot in 3d?
plot3(1,1,1)


% Draw global frame
drawFrame(joints(:,:,1))

% Draw all joints and connections
for i=2:size(T,3)
    % Draw line between previous joint and current joint
    drawLine(joints(:,:,i-1), joints(:,:,i))
    % Draw coordinate frame at joint
    drawFrame(joints(:,:,i))
end

% Set limits of build area
lb = -100;
ub = 500;
axis([lb,ub,lb,ub,lb,ub])

grid on
