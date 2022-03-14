function [theta, target] = sampleQuintic(coeffs, T, curTime)
    % SAMPLEQUINTIC   Sample the quintic interpolation at time `curTime`
    % 
    % ARGS
    % coeffs    : Coefficients of quintic for each theta and segment
    % T         : Time value for each via point
    % curTime   : Time to sample the interpolation
    %
    % RETURN
    % theta     : Theta values sampled at curTime
    % target    : Next via point

    % Detect which segment it is in
    seg = find(T>=curTime,1)-1;
    if seg==0
        seg=1;
    end
    target = seg + 1;
    dT = curTime - T(seg);

    % Sample quintic
    row = 6*(seg-1)+1;
    theta = coeffs(row,:) + coeffs(row+1,:)*dT + coeffs(row+2,:)*dT^2 + coeffs(row+3,:)*dT^3 ...
            + coeffs(row+4,:)*dT^4 + coeffs(row+5,:)*dT^5;
end