function theta = sampleQuintic(coeffs, T, curTime)
% SAMPLECUBIC   Sample the quintic interpolation at time `curTime`
% 
% ARGS
% coeffs    : Coefficients of quintic for each theta and segment
% T         : Time value for each via point
% curTime   : Time to sample the interpolation
%
% RETURN
% theta     : Theta values sampled at curTime

% Detect which segment it is in
seg = find(T>curTime,1)-1;
dT = curTime - T(seg);

% Sample cubic
row = 6*(seg-1)+1;
theta = coeffs(row,:) + coeffs(row+1,:)*dT + coeffs(row+2,:)*dT^2 + coeffs(row+3,:)*dT^3 ...
        + coeffs(row+3,:)*dT^4 + coeffs(row+4,:)*dT^5;
end