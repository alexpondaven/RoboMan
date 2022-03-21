function accel = sampleQuinticAcc(coeffs, T, curTime)
    %    Sample the acceleration of the quintic interpolation at time `curTime`
    % 
    % ARGS
    % coeffs    : Coefficients of quintic for each theta and segment
    % T         : Time value for each via point
    % curTime   : Time to sample the interpolation
    %
    % RETURN
    % accel  : acceleration values sampled at curTime
    
    % Detect which segment it is in
    if curTime > T(end)
        % fprintf("[sampleQuinticVel] curTime %0.4f greater than Tend %0.4f\n", curTime, T(end));
        curTime = T(end);
    end
    
    seg = find(T>=curTime,1)-1;
    if seg==0
        seg=1;
    end
    dT = curTime - T(seg);
    % fprintf("dT seg: %d QuinticVel: %d Curr_Time: %d\n", seg, dT, curTime);

    % Sample accel
    row = 6*(seg-1)+1;
    accel = 2*coeffs(row+2,:) + 6*coeffs(row+3,:)*dT ...
            + 12*coeffs(row+4,:)*dT^2 + 20*coeffs(row+5,:)*dT^3;
end
