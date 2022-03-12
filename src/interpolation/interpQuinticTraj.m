function [coeffs,T] = interpQuinticTraj(vias, Tend)
% INTERPTRAJ    Quintic time-based interpolation between given via points
%
% A quintic is fit between every pair of via points with constraints:
% - Time at each joint
% - Velocity equal before and after each joint
% 
% ARGS
% vias      : Via points (4 theta values each) that trajectory must pass through
% Tend      : Time to reach final via point
% 
% RETURNS
% coeffs    : Each joint's coefficients in cubic
%             theta(t) = a_0+a_1*t+a_2^2*t^2+a_3*t^3+a_4*t^4+a*5*t^5

% For now set via point times linearly
k = size(vias,1)-1; % Number of segments
T = Tend * cumsum([0,ones(1,k)])/k;
dT = diff(T); % Even spacing of times at each via point

% Size of linear problem
ncoeff = 6;
nconstr = 5;

% Interpolate for all 4 theta values (each column of vias)
coeffs = [];
for joint=1:4
    theta = vias(:,joint);
    
    % Create simultaneous equations
    % x = [a0; a11; a12; a13; a14; a15; a20; ... ; ak0; ak1; ak2; ak3; ak4; ak5]
    
    A = zeros(nconstr*k,ncoeff*k);
    b = zeros(nconstr*k,1); % 4 rows (constraints) for each segment
    
    % Add equations
    for seg=1:k
        r = nconstr*(seg-1) + 1;
        c = ncoeff*(seg-1) + 1;
        t = dT(seg);
        % Initial position
        A(r, c) = 1;
        b(r) = theta(seg);
        % Velocity continuity
        A(r+1, c+1:c+5) = [1, 2*t, 3*t^2, 4*t^3, 5*t^4];
        if seg~=k
            A(r+1,c+7) = -1;
        end
        % Acceleration continuity
        A(r+2, c+2:c+5) = [2, 6*t, 12*t^2, 20*t^3];
        if seg~=k
            A(r+2,c+8) = -2;
        end
        % Jerk continuity
        A(r+3, c+3:c+5) = [6, 24*t, 60*t^2];
        if seg~=k
            A(r+3,c+9) = -6;
        end
        % Final position
        A(r+4, c:c+5) = [1, t, t^2, t^3, t^4, t^5];
        b(r+4) = theta(seg+1);
    end
    
    % Add initial constraints: velocity=0 and acceleration=0
    % Replace 2nd to last equation with a11 = 0
    A(nconstr*k-1,:) = zeros(1,ncoeff*k);
    A(nconstr*k-1, 2) = 1;
    
    % Add equation to set a12=0
    A(nconstr*k+1,:) = zeros(1,ncoeff*k);
    A(nconstr*k+1, 3) = 1;
    b(nconstr*k+1) = 0;
    
    % Solve
    coeffs = [coeffs pinv(A)*b];
    
end

end