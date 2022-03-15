function [theta, target] = sampleQuinticVel(coeffs, T, curTime)
    %    Sample the velocity of the quintic interpolation at time `curTime`
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
    if curTime > T(end)
        % fprintf("[sampleQuinticVel] curTime %0.4f greater than Tend %0.4f\n", curTime, T(end));
        curTime = T(end);
    end
    
    seg = find(T>=curTime,1)-1;
    if seg==0
        seg=1;
    end
    target = seg + 1;
    dT = curTime - T(seg);
    % fprintf("dT seg: %d QuinticVel: %d Curr_Time: %d\n", seg, dT, curTime);

    % Sample vel
    row = 6*(seg-1)+1;
    theta = coeffs(row+1,:) + 2*coeffs(row+2,:)*dT + 3*coeffs(row+3,:)*dT^2 ...
            + 4*coeffs(row+4,:)*dT^3 + 5*coeffs(row+5,:)*dT^4;
end
