function [T, coeffs, Tend] = assignViaTimes2(vias, strat)
% assignViaTimes Assign times to each via point according to strategy
% and come up with coefficients
%
% Args
% vias  : Waypoints / via points of each theta (4 values in each row)
% strat : Strategy includes ['lin','dpos','acc','dvel']
%
% Return
% T     : Time of each via point (should be same #rows as vias)
% 
% Tend  : Time to reach last via point

k = size(vias,1)-1;     % Number of spacings to put in 

% For now, T is a vector from 0-1.
if strat=="lin"
    %% Space linearly strategy
    T = cumsum([0,ones(1,k)])/k;

elseif strat=="dpos"
    %% Heuristic based - change in position
    % Assign more time when position changes a lot
    maxPosDiff = max(abs(diff(vias)),[],2);
    timeProp = maxPosDiff / sum(maxPosDiff);
    T = [0; cumsum(timeProp)]';

elseif strat=="acc"
    %% Heuristic based acceleration
    % Acceleration heuristic is 1D Laplace filter [1, -2, 1]
   
    accHeur = zeros(size(vias));
    for joint=1:size(vias,2)
        theta = vias(:,joint);
    
        for i=1:k+1
            if i==1
                window = [theta(i); theta(i); theta(i+1)];
            elseif i==k+1
                window = [theta(i-1); theta(i); theta(i)];
            else
                window = theta(i-1:i+1);
            end
            laplace = [1 -2 1] * window;    % Take the dot product
            accHeur(i,joint) = laplace;
        end
    end

    accHeurNorm = zeros(k, size(vias,2));
    for i=1:k
        % Take the mean of pointwise accelerations
        % no need to divide by 2, we normalise later anyway
        accHeurNorm(i,:) = accHeur(i,:) + accHeur(i+1,:);
    end

    % Take max acceleration of each theta to determine how to space in time
    maxAcc = max(abs(accHeurNorm),[],2);
    normAcc = maxAcc / sum(maxAcc);

    % Assign more time to higher accelerations
    offset = 2/k;        % todo param to be tuned
    timeProp = normAcc + offset;
    normT = timeProp / sum(timeProp);
    T = [0; cumsum(normT)]';
   
elseif strat=="dvel"
    %% Heuristic based - velocity change
    % Velocity heuristic is 1D Laplace filter [-1, 1]
    
    velHeur = zeros(k+1,4);
    for joint=1:4
        theta = vias(:,joint);
      
        for i=1:k+1
            if i==1
                window = [theta(i); theta(i)];
            else
                window = theta(i-1:i);
            end
            laplace = [-1 1] * window;
            velHeur(i,joint) = laplace;
        end
    end

    % Take max acceleration of each theta to determine how to space in time
    accHeur = abs(diff(velHeur));
    maxAcc = max(accHeur,[],2);
    
    normAcc = maxAcc / sum(maxAcc);

    % Assign more time to higher accelerations
    offset = 0.1;        % param to be tuned
    timeProp = normAcc + offset;
    normT = timeProp / sum(timeProp);
    T = [0; cumsum(normT)]';
else
    sprintf("%s is not a valid strategy to assign via point times",strat )
    return
end

% Assign quintic coefficients here first.
coeffs = interpQuinticTraj(vias, T);

N_SAMPLES = 100;
dt = 1/N_SAMPLES;
tvec = 0:dt:1;

% Sample vel and acc:
accVect = zeros(N_SAMPLES, size(vias,2));
velVect = zeros(N_SAMPLES, size(vias,2));
for idx=1:N_SAMPLES
    accVect(idx,:) = sampleQuinticAcc(coeffs, T, tvec(idx));
    [velVect(idx,:), ~] = sampleQuinticVel(coeffs, T, tvec(idx));
end

% Obtain the maximum vel, acc experienced
peakVel = max(abs(velVect));
% Obtain the maximum acceleration experienced
peakAcc = max(abs(accVect));

% Scale Tend based on this value
% Convert (scaled) RPM to ticks/sec
velocityLimitTicks = (getDXLSettings().velocityLimit * 0.229) * (4096) / 60;
Tend = max(peakVel) / (0.7 * velocityLimitTicks);

% Alternatively, scale Tend based on peak acceleration
% MAX_ACCEL = 80;
% Tend = MAX_ACCEL / max(peakAcc);

T = T*Tend;

% Lazy to transform coefficients -> Run quintic interp again
coeffs = interpQuinticTraj(vias, T);

sprintf("T with strategy %s. Tend = %0.4f", strat, Tend);
T
end