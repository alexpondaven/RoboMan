function theta = inverseKin(x,y,z,theta_G,gripSize)
    % constants
    L1 = 128;   % capitalize L to make it look diff from 1
    L2 = 24;
    L3 = 124;
    L4 = 126;
    alpha_3 = atan(L2/L1);
    R_3 = sqrt(L2^2+L1^2);

    theta(1) = atan2(y,x);

    % Input G1
    g1_x = sqrt(x^2+y^2);
    g1_y = z - 77;

    % Calculating G2
    x_ = g1_x - L4*cos(theta_G);
    y_ = g1_y - L4*sin(theta_G);
    
    s3_a = (x_^2 + y_^2 - L3^2 - L2^2 - L1^2)/(2*L3*R_3);
    if abs(s3_a) > 1
        disp("invalid theta 3 value!")
        s3_a=1;
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
    end
    c2_a = sqrt(1-s2_a^2); % +- choose +ve now
    
    theta(2) = atan2(s2_a, c2_a) - alpha_2;

    theta(4) = theta_G - theta(2) - theta(3);

    theta(5) = gripSize;
    
    % check if any theta is out of bounds
    
    % Check where the rest of the joints are to prevent collision
    
end