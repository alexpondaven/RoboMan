function [jointVel, errorIntegral] = feedforwardPIcontrol(desiredVel, error, prevError)
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

    % TODO Tune this, PI control is currently unused

    Kp = 1;
    Ki = 0;     % Kp and Ki are scalars with values to be tuned

    errorIntegral = prevError + error;
    jointVel = desiredVel + Kp*error + Ki*errorIntegral;

end