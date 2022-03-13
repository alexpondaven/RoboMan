function T = assignViaTimes(vias, Tend)
 
    k = size(vias,1)-1;

    %% Space linearly strategy
    T = Tend * cumsum([0,ones(1,k)])/k;

    %% Heuristic based - change in position
    % Assign more time when position changes a lot
%     maxPosDiff = max(abs(diff(vias)),[],2);
%     timeProp = Tend * maxPosDiff / sum(maxPosDiff);
%     T = [0; cumsum(timeProp)]';

    
    %% Heuristic based acceleration
    % Acceleration heuristic is 1D Laplace filter [-1, 1]
    
    accHeur = zeros(k+1,4);
    for joint=1:4
        theta = vias(:,joint);
      
        for i=1:k+1
            if i==1
                window = [theta(i); theta(i)];
            else
                window = theta(i-1:i);
            end
            laplace = [-1 1] * window;
            accHeur(i,joint) = laplace;
        end
    end

   % Take max acceleration of each theta to determine how to space in time
   accHeur = abs(diff(accHeur))
   maxAcc = max(accHeur,[],2);
   
   normAcc = maxAcc / sum(maxAcc)

   % Assign more time to higher accelerations
   offset = 0.1;        % param to be tuned
    timeProp = normAcc + offset;
    normT = Tend * timeProp / sum(timeProp);
    T = [0; cumsum(normT)]';
   

    
end