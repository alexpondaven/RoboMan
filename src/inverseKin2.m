function [theta, ec] = inverseKin2(x,y,z,theta_G,gripOpen)
    % inverse kinematic function
    % Args:
    % x        : x-coordinate of end effector
    % y        : y-coordinate of end effector
    % z        : z-coordinate of end effector
    % theta_G  : orientation of robot wrist wrt to global frame
    % gripOpen : bool (open/closed)
    %
    % Returns:
    % theta : 5-vector of each servo angle in radians
    % ec    : return code with function status.
    %         use `IK_ErrorCodes` to interpret this
    
    % constants
    armDims = getArmDimensions();
    L1 = armDims.L1;
    L2 = armDims.L2;
    L3 = armDims.L3;
    L4 = armDims.L4;           % Alter this parameter for end effector grip position
    alpha_3 = armDims.alpha_3;
    R_3 = armDims.R_3;

    % joint limits
    jointBounds = getJointBounds();
    
    %% Calculate IK angles
    theta(1) = atan2(y,x);          % hip angle

    % Input G1 (end effector position)
    g1_x = sqrt(x^2+y^2);           % goal pos in 2d side view
    g1_y = z - 77;

    % Calculating G2 (wrist joint position)
    x_ = g1_x - L4*cos(theta_G);
    y_ = g1_y - L4*sin(theta_G);
    
    % sin(theta3) + alpha
    s3_a = (x_^2 + y_^2 - L3^2 - L2^2 - L1^2)/(2*L3*R_3);
    if abs(s3_a) > 1
        % fprintf("invalid theta 3 value!\n")
        ec = -1;
        return
    end
    c3_a = sqrt(1-s3_a^2); % +- choose +ve now
    theta(3) = atan2(s3_a, c3_a) - alpha_3;
    % if theta(3) out of joint limit, try to choose the 'elbow down' config
    theta3_lb = jointBounds(1, 3);
    theta3_ub = jointBounds(2, 3);
    if (theta(3) > theta3_ub) || (theta(3) < theta3_lb)
        % fprintf("Previous theta(3) value %0.2f out of bounds. Attempting to convert to an elbow-down configuration.\n", ...
            % rad2deg( theta(3) ));
        theta(3) = atan2(s3_a, -c3_a) - alpha_3;
        % fprintf("theta(3) elbow-down configuration: %0.2f\n", ...
        %     rad2deg( theta(3) ));
    end
    
    k1 = L2 + L3*cos(theta(3));
    k2 = L1 + L3*sin(theta(3));
    alpha_2 = atan(k2/k1);
    R_2 = sqrt(k1^2 + k2^2);
    s2_a = y_/R_2;
    if abs(s2_a) > 1
        % fprintf("invalid theta 2 value!\n")
        ec = -2;
        return 
    end
    c2_a = sqrt(1-s2_a^2); % +- choose +ve now
    
    theta(2) = atan2(s2_a, c2_a) - alpha_2;
    % if theta(2) out of joint limit, try to choose the 'elbow down' config
    theta2_lb = jointBounds(1, 2);
    theta2_ub = jointBounds(2, 2);
    if (theta(2) > theta2_ub) || (theta(2) < theta2_lb)
        % fprintf("Previous theta(2) value %0.2f out of bounds. Attempting to convert to an elbow-down configuration.\n", ...
            % rad2deg( theta(2) ));
        theta(2) = atan2(s2_a, -c2_a) - alpha_2;
        % fprintf("theta(2) elbow-down configuration: %0.2f\n", ...
            % rad2deg( theta(2) ));
    end
    
    theta(4) = theta_G - theta(2) - theta(3);
    % we need the theta2,3 values to determine the wrist (theta4) angle
    
    if gripOpen==true
        theta(5) = deg2rad(232);    % Fully open
    else
        theta(5) = deg2rad(95);     % To grip cube
    end                             % Fully closed: 90
   
    %% Error handling
    % Check if any joint is out of bounds
%     jointLimitErr = zeros(5, 'uint8');
    % one-hot vector to keep track of out-of-bound joints
    for idx=1:5
        lb = jointBounds(1,idx);
        ub = jointBounds(2, idx);
        if  (theta(idx) > ub) || (theta(idx) < lb)
%             jointLimitErr(idx) = 1;
            % fprintf("Theta %d angle %0.2f out of bounds. UB: %0.2f, LB: %0.2f\n",...
            %     idx, rad2deg(theta(idx)), rad2deg(ub), rad2deg(lb));
            ec = -3;
            return
        end
    end
        
    % Check if any theta is out of bounds or for collisions between joints
    % Go through transformation matrix
    joints = getJointPositions(theta);
    jointPosErr = checkJointPositions(joints);
    
    for idx=1:length(jointPosErr)
        if jointPosErr(idx) ~= 0
            % fprintf("Joint %d position out of bounds. x: %0.2f, y: %0.2f z: %0.2f\n",...
            %     idx, joints(1, idx), joints(2, idx), joints(3, idx));
            ec = -4;
            return
        end
    end
    
    ec = 0; % no error
end