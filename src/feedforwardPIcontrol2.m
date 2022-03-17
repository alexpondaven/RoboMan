function [jointVel, errorIntegral] = feedforwardPIcontrol2(desiredVel, error, prevError)
% PI+Feedforward controller
% 
% Args:
% desiredVel : feedforward velocity term
% error      : current joint error
% prevError  : accumulated joint error for Integral term
% 
% Returns:
% jointVel      : output velocity terms
% errorIntegral : summing prevError and error

    % TODO Tune Ki / Kp

    Kp = 0.2;
    Ki = 0.005;     % Kp and Ki are scalars with values to be tuned
    Kd = 0.0;

    errorIntegral = prevError + error;
    errorDerivative = 
    jointVel = desiredVel + Kp*error + Ki*errorIntegral;

end