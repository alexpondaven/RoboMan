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
    otherwise
        fprintf('[IK] Unknown error code: %d\n', ec)

    end
end