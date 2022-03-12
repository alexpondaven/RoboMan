function [coeffs,T] = interpCubicTraj(vias, Tend)
% INTERPTRAJ    Cubic time-based interpolation between given via points
%
% A cubic is fit between every pair of via points with constraints:
% - Time at each joint
% - Velocity equal before and after each joint
% 
% ARGS
% vias      : Via points (4 theta values each) that trajectory must pass through
% Tend      : Time to reach final via point
% 
% RETURNS
% coeffs    : Each joint's coefficients in cubic
%             theta(t) = a_0+a_1*t+a_2^2*t^2+a_3*t^3

% For now set via point times linearly
k = size(vias,1)-1; % Number of segments
T = Tend * cumsum([0,ones(1,k)])/k;
dT = diff(T); % Even spacing of times at each via point


% Interpolate for all 4 theta values (each column of vias)
coeffs = [];
for joint=1:4
    theta = vias(:,joint);
    
    % Create simultaneous equations
    % x = [a0; a11; a12; a13; a20; ... ; ak0; ak1; ak2; ak3]
    
    A = zeros(4*k);
    b = zeros(4*k,1);
    
    % Add equations
    for j=1:k
        i = 4*(j-1) + 1;
        % Initial position
        A(i, i) = 1;
        b(i) = theta(j);
        % Velocity continuity
        A(i+1, i+1:i+3) = [1, 2*dT(j), 3*dT(j)^2];
        if j~=k
            A(i+1,i+5) = -1;
        end
        % Acceleration continuity
        A(i+2, i+2:i+3) = [2, 6*dT(j)];
        if j~=k
            A(i+2,i+6) = -2;
        end
        % Final position
        A(i+3, i:i+3) = [1, dT(j), dT(j)^2, dT(j)^3];
        b(i+3) = theta(j+1);
    end
    
    % Replace 2nd to last equation with a11 = 0
    A(4*k-1,:) = zeros(1,4*k);
    A(4*k-1, 2) = 1;
    
    % Solve
    coeffs = [coeffs pinv(A)*b];
    
end

end