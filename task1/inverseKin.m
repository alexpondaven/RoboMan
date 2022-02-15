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

    
    theta(4) = 0;
    theta(5) = gripSize;
    
end