function theta = inverseKin(x,y,z,gripSize)
    theta(1) = atan2(y,x);
%     theta(1)=0;
    
    % Solve 2DOF problem 
    x_ = sqrt(x^2+y^2);
    y_ = z - 77;
    l1 = 130;
    l2 = 124;

    c2 = (x_^2+y_^2-l1^2-l2^2)/(2*l1*l2);
    s2 = sqrt(1-c2^2); % +- choose +ve now
    k1 = l1+l2*c2;
    k2 = l2*s2;
    theta_a = atan2(x_,y_) - atan2(k2,k1);
    theta_b = atan2(s2,c2);
    phi = atan(24/128);
%     theta(2) = -(pi/2 - theta_a - phi);
    theta(2) = theta_a + phi; 
    theta(3) = pi/2 - theta_b;
    
    theta(4) = pi/2;
    theta(5) = gripSize;
    
    %% TY's IK solution below
    x_ = sqrt(x^2+y^2);
    y_ = z - 77;
    L1 = 128;   % capitalize L to make it look diff from 1
    L2 = 24;
    L3 = 124;
    alpha_3 = atan(L2/L1);
    R_3 = sqrt(L2^2+L1^2);
    
    s3_a = (x_^2 + y_^2 - L3^2 - L2^2 - L1^2)/(2*L3*R_3);
    if abs(s3_a) > 1
        print("invalid theta 3 value!")
        return
    else
        c3_a = sqrt(1-s3_a^2); % +- choose +ve now
    end
    theta(3) = atan2(s3_a, c3_a) - alpha_3;
    
    k1 = L2 + L3*cos(theta(3));
    k2 = L1 + L3*sin(theta(3));
    alpha_2 = atan(k2/k1);
    R_2 = sqrt(k1^2 + k2^2);
    s2_a = y_/R_2;
    if abs(s2_a) > 1
        print("invalid theta 2 value!")
        return
    else
        c2_a = sqrt(1-s2_a^2); % +- choose +ve now
    end
    
    theta(2) = atan2(s2_a, c2_a) - alpha_2;
    
end