function T_i = getDHTransform(theta, a, alpha, d)
% Args:
% theta : rotation of joint
% a     : previous joint length
% alpha : angle between joint axes
% d     : distance between links along z
% Returns: 4x4 Transformation Matrix

    % Compute DH transform
    T_i = [cos(theta)               -sin(theta)             0               a;
            sin(theta)*cos(alpha)   cos(theta)*cos(alpha)   -sin(alpha)     -sin(alpha)*d;
            sin(theta)*sin(alpha)   cos(theta)*sin(alpha)   cos(alpha)      cos(alpha)*d;
            0                       0                       0               1];
end