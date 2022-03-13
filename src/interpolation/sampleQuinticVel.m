function [theta, target] = sampleQuinticVel(coeffs, T, curTime)
    % SampleQuinticVel Sample the velocity of the quintic interpolation at time `curTime`
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
    seg = find(T>curTime,1)-1;
    target = seg + 1;
    dT = curTime - T(seg);
    
    % Sample vel
    row = 6*(seg-1)+1;
    theta = coeffs(row+1,:) + 2*coeffs(row+2,:)*dT + 3*coeffs(row+3,:)*dT^2 ...
            + 4*coeffs(row+4,:)*dT^3 + 5*coeffs(row+5,:)*dT^4;
end
