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
    armDims = armDimensions();
    L1 = armDims.L1;
    L2 = armDims.L2;
    L3 = armDims.L3;
    L4 = armDims.L4;           % Alter this parameter for end effector grip position
    alpha_3 = armDims.alpha_3;
    R_3 = armDims.R_3;

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
        disp("invalid theta 3 value!")
        ec = -1;
        return
    end
    c3_a = sqrt(1-s3_a^2); % +- choose +ve now
    theta(3) = atan2(s3_a, c3_a) - alpha_3;
    
    k1 = L2 + L3*cos(theta(3));
    k2 = L1 + L3*sin(theta(3));
    alpha_2 = atan(k2/k1);
    R_2 = sqrt(k1^2 + k2^2);
    s2_a = y_/R_2;
    if abs(s2_a) > 1
        disp("invalid theta 2 value!")
        ec = -2;
        return 
    end
    c2_a = sqrt(1-s2_a^2); % +- choose +ve now
    
    theta(2) = atan2(s2_a, c2_a) - alpha_2;
    theta(4) = theta_G - theta(2) - theta(3);
    % we need the theta2,3 values to determine the wrist (theta4) angle
    
    if gripOpen==true
        theta(5) = deg2rad(232);    % Fully open
    else
        theta(5) = deg2rad(95);     % To grip cube
    end                             % Fully closed: 90
   
    % Check if any theta is out of bounds or for collisions between joints
    % Go through transformation matrix
    
    
    
    ec = 0; % no error
end