function [T,Tend] = assignViaTimes(vias, strat)
% assignViaTimes Assign times to each via point according to strategy
%
% Args
% vias  : Waypoints / via points of each theta (4 values in each row)
% strat : Strategy includes ['lin','dpos','acc','dvel']
%
% Return
% T     : Time of each via point (should be same #rows as vias)
% Tend  : Time to reach last via point


% Determine Tend proportional to the max total distance travelled
maxTotalDist = max(sum(abs(diff(vias))));
VEL_SCALING = 0.175;      % TODO Tune this
maxVel = (getDXLSettings().velocityLimit/60*0.229*4096) * VEL_SCALING;
% Convert (scaled) RPM to Ticks/second and apply scaling
Tend = maxTotalDist / maxVel;
Tend = min( 50, max( 1, Tend ))  % floor and ceiling these
% Tend = 2;  % fr now

k = size(vias,1)-1;

if strat=="lin"
    %% Space linearly strategy
    T = Tend * cumsum([0,ones(1,k)])/k;

elseif strat=="dpos"
    %% Heuristic based - change in position
    % Assign more time when position changes a lot
    maxPosDiff = max(abs(diff(vias)),[],2);
    timeProp = Tend * maxPosDiff / sum(maxPosDiff);
    T = [0; cumsum(timeProp)]';

elseif strat=="acc"
    %% Heuristic based acceleration
    % Acceleration heuristic is 1D Laplace filter [1, -2, 1]
   
    accHeur = zeros(k+1,4);
    for joint=1:4
        theta = vias(:,joint);
    
        for i=1:k+1
            if i==1
                window = [theta(i); theta(i); theta(i+1)];
            elseif i==k+1
                window = [theta(i-1); theta(i); theta(i)];
            else
                window = theta(i-1:i+1);
            end
            laplace = [1 -2 1] * window;
            accHeur(i,joint) = laplace;
        end
    end

    accHeurNorm = zeros(k,4);
    for i=1:k
        % Take the mean of both accelerations
        % no need to divide by 2, we normalise later anyway
        accHeurNorm(i,:) = accHeur(i,:) + accHeur(i+1,:);
    end

    % Take max acceleration of each theta to determine how to space in time
    maxAcc = max(abs(accHeurNorm),[],2);
    normAcc = maxAcc / sum(maxAcc);

    % Assign more time to higher accelerations
    offset = 0.1;        % param to be tuned
    timeProp = normAcc + offset;
    normT = Tend * timeProp / sum(timeProp);
    T = [0; cumsum(normT)]';
   
elseif strat=="acc2"
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
    normT = Tend * timeProp / sum(timeProp);
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
    normT = Tend * timeProp / sum(timeProp);
    T = [0; cumsum(normT)]';
else
    sprintf("%s is not a valid strategy to assign via point times",strat )
    return
end

sprintf("T with strategy %s. Tend = %0.4f", strat, Tend);
T
end