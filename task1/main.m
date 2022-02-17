close all
clear all
% Array of controlled angles - ith angle is rotation of ith servo
% Last parameter is how open the gripper is
% theta = [5,15,29,345,20];

%% Calculate theta for given x,y,z
% syms theta [1 4]
% Symbolic
% T1 = eye(4); 
% % Base of robot s.t. x axis pointing into link
% T2 = DHTransform(struct("theta",   theta1, ... % Base of first servo
%                             "alpha",    0, ... 
%                             "a",        0, ...
%                             "d",        34));
% T3 = DHTransform(struct("theta",  pi/2, ... % Intermediate
%                             "alpha",    pi/2, ... 
%                             "a",        0, ... 
%                             "d",        0)); 
% T4 = DHTransform(struct("theta",   theta2, ... % Base of second servo
%                             "alpha",    0, ... 
%                             "a",        43, ... 
%                             "d",        0)); 
% T5 = DHTransform(struct("theta",   -pi/2, ... % Elbow between servos
%                             "alpha",    0, ... 
%                             "a",        128, ... 
%                             "d",        0)); 
% T6 = DHTransform(struct("theta",   theta3, ...  % Base of third servo
%                             "alpha",    0, ... 
%                             "a",        24, ... 
%                             "d",        0));
% T7 = DHTransform(struct("theta",   theta4, ... % Base of fourth servo
%                             "alpha",    0, ... 
%                             "a",        124, ... 
%                             "d",        0)); 
% T8 = DHTransform(struct("theta",   0, ...% End effector
%                             "alpha",    0, ... 
%                             "a",        126, ... 
%                             "d",        0)); 
% 
% T = T1*T2*T3*T4*T5*T6*T7*T8;

theta = inverseKin(0,150,0,10);
coordFrames(theta)

%% Draw robot arm

% Move all angles
% while 1
%     for i=1:2*pi
%         coordFrames([i,i,i,i,20]);
%     end
% end

% Draw static default position
% coordFrames([0,0,0.1,0,10])