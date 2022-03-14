function jointPos = getCurrEndpointCoords(currServoAngles)
% Returns current endpoint x,y,z,theta_g coordinates based on currrent servo ticks
% Args:
% currServoAngles : 4-vector of servo angles in ticks
% 
% Returns:
% jointPos : endpoint coordinates calculated via forward kinematics.
%            x,y,z in mm, theta_g in radians

    jointParams = getJointParams();

    theta = zeros(1,4);
    for i=1:4
        theta(i) = (currServoAngles(i) * pi / 2048);
        theta(i) = theta(i) - jointParams(i).offset;
        theta(i) = theta(i) * jointParams(i).invert;
    end
    indivJointCoords = getJointPositions(theta);

    % xyz coords of end effector is just the last point
    jointPos(1:3) = indivJointCoords(1:3,end);

    % find theta_g
    jointPos(4) = theta(2) + theta(3) + theta(4);

end