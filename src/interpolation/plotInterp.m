function plotInterp(vias, coeffs, T)
% PLOTINTERP    Plot cubic interpolation of via points from cubic
% coefficients of each segment.
%
% ARGS
% vias      : Theta values of via points
% coeffs    : Coefficients of cubic for each theta and segment
% T         : Time value for each via point
clf
close
dT = diff(T);
k = size(dT,2);
for joint=1:4
    theta = vias(:,joint);
    subplot(2,2,joint)
    dt = 0.01;
    t = [];
    interp = [];

    for j=1:k
        i = 4*(j-1) + 1;
        jt = 0:dt:dT(j);
        jInterp = coeffs(i,joint) + coeffs(i+1,joint)*jt + coeffs(i+2,joint)*jt.^2 + coeffs(i+3,joint)*jt.^3;
        jt = jt + T(j);
        
        t = [t jt];
        interp = [interp jInterp];
    end

    plot(t,interp,'k')
    title("Theta " + joint)
    xlabel("time")
    ylabel("theta")
    % Plot via points
    hold on

    for via=1:k+1
        hold on
        plot(T(via), theta(via),'ro')
    end
    sgtitle("Cubic interpolation between via points")
end
end