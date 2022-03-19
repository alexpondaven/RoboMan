function admissible = checkOGAdmissibility(ogCoords)
% Checks whether a particular occupancy grid coordinate can be reached by the robot arm
% Args:
% ogCoords : 4-vector of (theta_g, theta_1, x', y') occupancy grid indices.
%
% Returns:
% admissible : boolean whether the desired coordinate is admissible or not.

    cartesian = OGToCartesianCoords(ogCoords);
    xVal = cartesian(1);
    yVal = cartesian(2);
    zVal = cartesian(3);
    theta_g = cartesian(4);
    
    [~, ec] = inverseKin2(xVal, yVal, zVal, theta_g, false);

    admissible = ec==0;

    if ~admissible
        IK_ErrorCodes(ec);  % print if there's an error
    end
end

