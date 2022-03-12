function IK_ErrorCodes(ec)
% Translates numeric error codes from `inverseKin2` function.
% Args:
% ec : error code returned from `inverseKin2`

    switch ec
    case 0
        disp('[IK] no errors.')
    case -1
        disp('[IK] sin(theta3...) > 1.')
    case -2
        disp('[IK] sin(theta2...) > 1.')
    case -3
        disp('[IK] Joint angle out of bound.')
    case -4
        disp('[IK] Joint location out of bound.')
    case -5
        disp('[IK] calculated final joint position does not match up with joint target.')
    otherwise
        fprintf('[IK] Unknown error code: %d\n', ec)

    end
end