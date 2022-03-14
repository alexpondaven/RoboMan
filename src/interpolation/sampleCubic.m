function theta = sampleCubic(coeffs, T, curTime)
% SAMPLECUBIC   Sample the cubic interpolation at time `curTime`
% 
% ARGS
% coeffs    : Coefficients of cubic for each theta and segment
% T         : Time value for each via point
% curTime   : Time to sample the interpolation
%
% RETURN
% theta     : Theta values sampled at curTime

% Detect which segment it is in
seg = find(T>=curTime,1)-1;
if seg==0
    seg=1;
end
dT = curTime - T(seg);

% Sample cubic
row = 4*seg-3;
theta = coeffs(row,:) + coeffs(row+1,:)*dT + coeffs(row+2,:)*dT^2 + coeffs(row+3,:)*dT^3;
end