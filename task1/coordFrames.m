function [T, joints] = coordFrames(theta)
    
    %% Transformations
    
    % Transform list
    % T = zeros(4,4,numJoints);
    
    % World frame
    T(:,:,1) = eye(4); 
    % Base of robot s.t. x axis pointing into link
    T(:,:,2) = DHTransform(struct("theta",   theta(1), ... % Base of first servo
                                "alpha",    0, ... 
                                "a",        0, ...
                                "d",        34));
    T(:,:,3) = DHTransform(struct("theta",  pi/2, ... % Intermediate
                                "alpha",    pi/2, ... 
                                "a",        0, ... 
                                "d",        0)); 
    T(:,:,4) = DHTransform(struct("theta",   theta(2), ... % Base of second servo
                                "alpha",    0, ... 
                                "a",        43, ... 
                                "d",        0)); 
    T(:,:,5) = DHTransform(struct("theta",   -pi/2, ... % Elbow between servos
                                "alpha",    0, ... 
                                "a",        128, ... 
                                "d",        0)); 
    T(:,:,6) = DHTransform(struct("theta",   theta(3), ...  % Base of third servo
                                "alpha",    0, ... 
                                "a",        24, ... 
                                "d",        0));
    T(:,:,7) = DHTransform(struct("theta",   theta(4), ... % Base of fourth servo
                                "alpha",    0, ... 
                                "a",        124, ... 
                                "d",        0)); 
    T(:,:,8) = DHTransform(struct("theta",   0, ...% End effector
                                "alpha",    0, ... 
                                "a",        126, ... 
                                "d",        0)); 
    
%     syms(sym('theta',[1,4]))
    
    %% Positions of joints
    % joints = zeros(1,numJoints);
    
    joints(:,:,1) = T(:,:,1);
%     symbolic = T(:,:,1);
    
    % Apply transforms
    for i=2:size(T,3)
        joints(:,:,i) = joints(:,:,i-1) * T(:,:,i);
%         symbolic = symbolic * 
    end
    
    %% Plot joints
    % Needed to display plot in 3d?
    plot3(1,1,1)
    title("3D view of robot arm")
    
    % Draw global frame
    drawFrame(joints(:,:,1),500)
    
    % Draw all joints and connections
    for i=2:size(T,3)
        % Draw line between previous joint position and current joint position
        drawLine(joints(1:3,4,i-1), joints(1:3,4,i))
        % Draw coordinate frame at joint
        drawFrame(joints(:,:,i), 20)
    end

    % Draw gripper
    drawGripper(joints(:,:,end), theta(5))

    % Set limits of build area
    ub = 500;
    axis([-100,ub,-ub,ub,0,ub])
    grid on

    % Plot the current angle
    drawnow
    % Add delay to avoid lag
%     pause(1)

%% Plot top-down (x,y) and side (x,z) (y,z) views of the arm
% Could plot these on same plot?
% % (x,y) view
% xy_plot = figure;
% title("XY View of Robot Arm");
% grid on
% hold on
% plot([0,500], [0,0], 'r', 'LineWidth', 0.75);  % x axis
% plot([0,0], [0,500], 'g', 'LineWidth', 0.75);  % y axis
% axis([-ub,ub,-100,ub]);
% 
% % (y,z) view
% yz_plot = figure;
% title("YZ View of Robot Arm");
% grid on
% hold on
% plot([0,500], [0,0], 'g', 'LineWidth', 0.75);  % y axis
% plot([0,0], [0,500], 'b', 'LineWidth', 0.75);  % z axis
% axis([-100,ub,0,ub]);
% 
% % (x,z) view
% xz_plot = figure;
% title("XZ View of Robot Arm");
% grid on
% hold on
% plot([0,500], [0,0], 'r', 'LineWidth', 0.75);  % x axis
% plot([0,0], [0,500], 'b', 'LineWidth', 0.75);  % z axis
% axis([-ub,ub,0,ub]);
% 
% for i=2:size(T,3)
%     % Draw line between previous joint position and current joint position
%     point1 = joints(1:3,4,i-1);
%     point2 = joints(1:3,4,i);
%     lbl = sprintf('joint %d', i);
%     
%     figure(xy_plot);
%     plot([point1(1) point2(1)], [point1(2) point2(2)], 'LineWidth', 3);
%     
%     figure(yz_plot);
%     plot([point1(2) point2(2)], [point1(3) point2(3)], 'LineWidth', 3);
%     
%     figure(xz_plot);
%     plot([point1(1) point2(1)], [point1(3) point2(3)], 'LineWidth', 3);
%     
%     % Draw coordinate frame at joint
% %     drawFrame(joints(:,:,i), 20)
% end
% 
% % Plot for all 3
% figure(xy_plot);
% hold off
% figure(yz_plot);
% hold off
% figure(xz_plot);
% hold off

end


