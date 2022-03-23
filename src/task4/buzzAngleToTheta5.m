function theta5 = buzzAngleToTheta5(buzzerAngle, theta1)
% BuzzAngleToTheta5  Convert the required buzzer angle into the appropriate theta5 rotation
% The gripper uses translational movement from opening/closing the 
% gripper mount (changing theta5) to rotate the buzzer
%
% Args:
% buzzerAngle : Rotation of the buzzer (0-2*pi)
% theta1      : Pointwise hip joint angle
%
% Returns:
% theta5      : Joint angle of servo 5 (opening/closing gripper)

    theta5_cmd = rad2deg(buzzerAngle - theta1);
    
    % set - 90 deg @ center (464)
    % set -180 deg @ (0)
    % set 0 deg @ (928)
    theta5 = theta5_cmd * (928/180) + 928;  %linear relationship, see onenote

    % error checking
    if theta5 > 928
        fprintf("[buzzAngleToTheta5] theta 5 above bounds: commanded %0.1fdeg, %d ticks\n", theta5_cmd, theta_5);
        theta5 = 928;
    elseif theta5 < 0
        fprintf("[buzzAngleToTheta5] theta 5 below bounds: commanded %0.1fdeg, %d ticks\n", theta5_cmd, theta_5);
        theta5 = 0;
    end

    % min -> 0   -> \ (closed)
    %     -> 287 -> -
    %     -> 508 -> /
    % max -> 928 -> | (open) 

    % wideOpen = 1160;        % dist:68
    % fullyClosed = 2575;     % dist:14

    % % From wideOpen to fullyClosed the arm traverses 68-14=54mm
    % % The buzzwire has diameter 13mm
    % % Turning it 180 deg = ~6.5*pi = 20.42mm
    % % (68-14)/(2575-1160) = 0.03816mm/tick
    % % 1mm gives ~180/20.42 deg/mm = 8.8149deg/mm
    % % 8.8149deg/mm * 0.03816mm/tick = 0.33638 deg/tick

    % % wideOpenAngle = 

    % theta5 = wideOpen; %for now

end

