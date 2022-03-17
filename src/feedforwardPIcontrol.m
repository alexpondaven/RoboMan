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

    % TODO Tune Ki / Kp

    Kp = 0.1;   % FOR MAINSERVOLOOP2
    Ki = 0;     % Kp and Ki are scalars with values to be tuned

    errorIntegral = prevError + error;
    jointVel = desiredVel + Kp*error + Ki*errorIntegral;

end