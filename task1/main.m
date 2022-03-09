close all
clear all
% Array of controlled angles - ith angle is rotation of ith servo
% Last parameter is how open the gripper is

%% Draw stacking
posPath = moveCube(1,5,[0,0,0,0,0,0]);

for i=1:size(posPath,1)
    theta = inverseKin(posPath(i,1),posPath(i,2),posPath(i,3),posPath(i,4),posPath(i,5));
    coordFrames(theta);
    pause(3)
end

%% Draw box
% z=50;
% corners = [150 -100 z
%             150 100 z
%             350 100 z
%             350 -100 z];
% while 1
%     % Draw square
% %     for y=-50:10:50
% %         theta = inverseKin(150,y,z,0,10);
% %         coordFrames(theta);
% %     end
% %     for x=150:10:200
% %         theta = inverseKin(x,50,z,0,10);
% %         coordFrames(theta);
% %     end
% %     for y=50:-10:-50
% %         theta = inverseKin(200,y,z,0,10);
% %         coordFrames(theta);
% %     end
% %     for x=200:-10:150
% %         theta = inverseKin(x,-50,z,0,10);
% %         coordFrames(theta);
% %     end
% %     for i=
%     
%     
%     posPath = genLineTraj(corners(1,:),corners(2,:));
%     
%     for i=1:size(posPath,1)
%         pos = posPath(i,:);
%         theta = inverseKin(pos(1),pos(2),pos(3),0,10);
%         coordFrames(theta);
%     end
% 
%     posPath = genLineTraj(corners(2,:),corners(3,:));
%     
%     for i=1:size(posPath,1)
%         pos = posPath(i,:);
%         theta = inverseKin(pos(1),pos(2),pos(3),0,10);
%         coordFrames(theta);
%     end
% 
%     posPath = genLineTraj(corners(3,:),corners(4,:));
%     
%     for i=1:size(posPath,1)
%         pos = posPath(i,:);
%         theta = inverseKin(pos(1),pos(2),pos(3),0,10);
%         coordFrames(theta);
%     end
% 
%     posPath = genLineTraj(corners(4,:),corners(1,:));
%     
%     for i=1:size(posPath,1)
%         pos = posPath(i,:);
%         theta = inverseKin(pos(1),pos(2),pos(3),0,10);
%         coordFrames(theta);
%     end
% end

% theta = inverseKin(150,0,0,0,10);
% coordFrames(theta);

%% Dynamixel theta
% theta = inverseKinDynamixel(150,0,50,10)

%% Draw robot arm

% Move all angles
% while 1
%     for i=1:2*pi
%         coordFrames([i,i,i,i,20]);
%     end
% end

% Draw static default position
% coordFrames([0,0,0.1,0,10])