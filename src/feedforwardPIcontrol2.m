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
    Kp = 0.2;
    Ki = 0.005;
    Kd = 0.0;

    errorIntegral = errorIntegral + error;
    errorDerivative = error - prevError; % We want the effect to be negative when error is decreasing

    pTerm = Kp*error;
    iTerm = Ki*errorIntegral;
    dTerm = Kd*errorDerivative;
    % Debugging
    for i=1:length(desiredVel)
        fprintf("[%d] err: %04d | errAcc: %04d | errDer: %04d | desiredVel: %04d | pTerm: %04d | iTerm: %04d | dTerm: %04d\n",...
        i, error(i), errorIntegral(i), errorDerivative(i), desiredVel(i), pTerm(i), iTerm(i), dTerm(i) );
    end

    jointVel = desiredVel + pTerm + iTerm + dTerm;

end