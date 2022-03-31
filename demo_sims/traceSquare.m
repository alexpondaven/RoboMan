close all
clear all
% Array of controlled angles - ith angle is rotation of ith servo
% Last parameter is how open the gripper is

%% Draw stacking
% posPath = moveCube(1,5,[0,0,0,0,0,0]);
% 
% for i=1:size(posPath,1)
%     theta = inverseKin(posPath(i,1),posPath(i,2),posPath(i,3),posPath(i,4),posPath(i,5));
%     coordFrames(theta);
%     pause(3)
% end

%% Draw box
numPoints = 20;
square_thickness = 2;
colours = ['r','g','b','c'];

% XY plane
z=50;
theta_g=-pi/2;
corners = [100,-50,z
            100,50,z
            200,50,z
            200,-50,z
            100,-50,z];

% XZ plane
% y = 50;
% theta_g=0;
% corners = [150,y,50
%             150,y,150
%             250,y,150
%             250,y,50
%             150,y,50];

% YZ plane
% x = 50;
% theta_g=0;
% corners = [x,200,50
%             x,200,150
%             x,300,150
%             x,300,50
%             x,200,50];

start_pos = corners(1,:);
prev_pos = start_pos;
while 1
    % Draw path
    done_segments = {};
    for i=1:size(corners,1)-1
        path = linearInterpolate(corners(i,:),corners(i+1,:),numPoints);
        for j=1:size(path,1)
            curr_pos = path(j,:);
            theta = inverseKin(curr_pos(1),curr_pos(2),curr_pos(3),theta_g,10);
            coordFrames(theta);
%             subplot(2,2,1)
            % Draw all fully square segments
            for k=1:size(done_segments,2)
                done_seg = done_segments{k};
                drawLine(done_seg(1,:), done_seg(2,:),colours(k),square_thickness);
            end
    
            % Draw current square segments
            drawLine(corners(i,:), curr_pos, colours(i), square_thickness);
            prev_pos = curr_pos;
            drawnow
        end
        done_segments{end+1} = [corners(i,:); corners(i+1,:)];
    end

    % Draw square
%     for y=-50:10:50
%         curr_pos = [100,y,z];
%         theta = inverseKin(curr_pos(1),curr_pos(2),curr_pos(3),theta_g,10);
%         coordFrames(theta);
%         
%         subplot(2,2,1)
%         drawLine(start_pos, curr_pos, 2);
% %         start_pos = curr_pos;
%         drawnow
%     end
%     for x=100:10:200
%         curr_pos = [x,50,z];
%         theta = inverseKin(curr_pos(1),curr_pos(2),curr_pos(3),theta_g,10);
%         coordFrames(theta);
% 
%         drawLine(prev_pos, curr_pos);
%         prev_pos = curr_pos;
%     end
%     for y=50:-10:-50
%         curr_pos = [x,50,z];
%         theta = inverseKin(curr_pos(1),curr_pos(2),curr_pos(3),theta_g,10);
%         coordFrames(theta);
% 
%         drawLine(prev_pos, curr_pos);
%         prev_pos = curr_pos;
%     end
%     for x=200:-10:100
%         curr_pos = [x,-50,z];
%         theta = inverseKin(curr_pos(1),curr_pos(2),curr_pos(3),theta_g,10);
%         coordFrames(theta);
% 
%         drawLine(prev_pos, curr_pos);
%         prev_pos = curr_pos;
%     end
    
    % Interpolating with sine wave (smooth motion)
%     posPath = genLineTraj(corners(1,:),corners(2,:));
%     
%     for i=1:size(posPath,1)
%         pos = posPath(i,:);
%         theta = inverseKin(pos(1),pos(2),pos(3),theta_g,10);
%         coordFrames(theta);
%     end
% 
%     posPath = genLineTraj(corners(2,:),corners(3,:));
%     
%     for i=1:size(posPath,1)
%         pos = posPath(i,:);
%         theta = inverseKin(pos(1),pos(2),pos(3),theta_g,10);
%         coordFrames(theta);
%     end
% 
%     posPath = genLineTraj(corners(3,:),corners(4,:));
%     
%     for i=1:size(posPath,1)
%         pos = posPath(i,:);
%         theta = inverseKin(pos(1),pos(2),pos(3),theta_g,10);
%         coordFrames(theta);
%     end
% 
%     posPath = genLineTraj(corners(4,:),corners(1,:));
%     
%     for i=1:size(posPath,1)
%         pos = posPath(i,:);
%         theta = inverseKin(pos(1),pos(2),pos(3),theta_g,10);
%         coordFrames(theta);
%     end
end

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