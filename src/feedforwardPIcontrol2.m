function [jointVel, errorIntegral] = feedforwardPIcontrol2(desiredVel, error, prevError, errorIntegral)
% PI+Feedforward controller
% 
% Args:
% desiredVel : feedforward velocity term
% error      : current joint error
% errorIntegral  : accumulated joint error for Integral term
% 
% Returns:
% jointVel      : output velocity terms
% errorIntegral : summing prevError and error

    % TODO Tune Ki / Kp / Kd values
    Kp = [4.0 6.0 7.0 6.0];
    Ki = [0.0 0.0 0.0 0.0];
    Kd = [1.0 15.0 15.0 6.0];

    errorIntegral = errorIntegral + error;
    errorDerivative = error - prevError; % We want the effect to be negative when error is decreasing

    pTerm = Kp.*error;
    iTerm = Ki.*errorIntegral;
    dTerm = Kd.*errorDerivative;
    % Debugging
    % for i=1:length(desiredVel)
    %     fprintf("[%d] err: %0.2f | errAcc: %0.2f | errDer: %0.2f | desiredVel: %0.2f | pTerm: %0.2f | iTerm: %0.2f | dTerm: %0.2f\n",...
    %     i, error(i), errorIntegral(i), errorDerivative(i), desiredVel(i), pTerm(i), iTerm(i), dTerm(i) );
    % end
    % fprintf('\n');

    jointVel = desiredVel + pTerm + iTerm + dTerm;

end