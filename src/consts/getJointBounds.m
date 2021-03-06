function jointBounds = getJointBounds()
% Returns (lower, upper) joint limits of each joint in radians.

    jointBounds(:, 1) = [deg2rad( -100), deg2rad( 100)];  % hip      (theta1)
    jointBounds(:, 2) = [deg2rad(-118), deg2rad(100)];  % shoulder (theta2)
    jointBounds(:, 3) = [deg2rad(-115), deg2rad( 90)];  % elbow    (theta3)
    jointBounds(:, 4) = [deg2rad(-110), deg2rad(120)];  % wrist    (theta4)
    jointBounds(:, 5) = [deg2rad(  90), deg2rad(232)];  % gripper  (theta5)
    % jointBounds(:, 5) = [deg2rad(   0), deg2rad( 82)];  % Rotator (Task 4)  (theta5)
end

