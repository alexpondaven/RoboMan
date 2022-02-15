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
    axis([-ub,ub,-100,ub,0,ub])
    grid on

    % Plot the current angle
    drawnow
    % Add delay to avoid lag
%     pause(1)
end


