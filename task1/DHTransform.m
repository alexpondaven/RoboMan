function T_i = DHTransform(param)
    % param.theta - rotation of joint
    % param.a - previous joint length
    % param.alpha - angle between joint axes
    % param.d - distance between links along z
    theta = deg2rad(param.theta);
    alpha = deg2rad(param.alpha);
    a = param.a;
    d = param.d;
    % Compute DH transform
    T_i = [cos(theta)               -sin(theta)             0               a;
            sin(theta)*cos(alpha)   cos(theta)*cos(alpha)   -sin(alpha)     -sin(alpha)*d;
            sin(theta)*sin(alpha)   cos(theta)*sin(alpha)   cos(alpha)      cos(alpha)*d;
            0                       0                       0               1];
end