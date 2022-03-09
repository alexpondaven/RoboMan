function T_i = DHTransform(param)
% Args:
% param.theta : rotation of joint
% param.a     : previous joint length
% param.alpha : angle between joint axes
% param.d     : distance between links along z
% Returns: 4x4 Transformation Matrix

    theta = param.theta;
    alpha = param.alpha;
    a = param.a;
    d = param.d;
    % Compute DH transform
    T_i = [cos(theta)               -sin(theta)             0               a;
            sin(theta)*cos(alpha)   cos(theta)*cos(alpha)   -sin(alpha)     -sin(alpha)*d;
            sin(theta)*sin(alpha)   cos(theta)*sin(alpha)   cos(alpha)      cos(alpha)*d;
            0                       0                       0               1];
end