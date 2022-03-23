function plotQuinticInterp(vias, coeffs, T)
    % PLOTINTERP    Plot quintic interpolation of via points from quintic
    % coefficients of each segment.
    %
    % ARGS
    % vias      : Theta values of via points
    % coeffs    : Coefficients of cubic for each theta and segment
    % T         : Time value for each via point
    % clf
    % close
    dT = diff(T);
    k = size(dT,2);
    numJoints = size(vias,2);
    for joint=1:numJoints
        theta = vias(:,joint);
        % subplot(2,2,joint)
        dt = 0.01;
        t = [];
        interp = [];
        v_interp = [];
        a_interp = [];
    
        for j=1:k
            i = 6*(j-1) + 1;
            jt = 0:dt:dT(j);
            jInterp = coeffs(i,joint) + coeffs(i+1,joint)*jt + coeffs(i+2,joint)*jt.^2 + coeffs(i+3,joint)*jt.^3 + coeffs(i+4,joint)*jt.^4 + coeffs(i+5,joint)*jt.^5;
            vInterp = coeffs(i+1,joint) + 2*coeffs(i+2,joint)*jt + 3*coeffs(i+3,joint)*jt.^2 + 4*coeffs(i+4,joint)*jt.^3 + 5*coeffs(i+5,joint)*jt.^4;
            aInterp = 2*coeffs(i+2,joint) + 6*coeffs(i+3,joint)*jt + 12*coeffs(i+4,joint)*jt.^2 + 20*coeffs(i+5,joint)*jt.^3;
            jt = jt + T(j);
            
            t = [t jt];
            interp = [interp jInterp];
            v_interp = [v_interp vInterp];
            a_interp = [a_interp aInterp];
        end
        
        % Plot lines
        subplot(numJoints,3, 3*(joint-1)+1)     % position
        plot(t, interp)
        hold on
        title("Theta " + joint + " position")
        for via=1:k+1
            plot(T(via), theta(via),'ro')
        end
        grid on
        
        subplot(numJoints,3, 3*(joint-1)+2)     % velocity
        plot(t,v_interp)
        hold on
        title("Theta " + joint + " velocity")
        grid on
        
        subplot(numJoints,3, 3*(joint-1)+3)     % acceleration
        plot(t,a_interp)
        hold on
        title("Theta " + joint + " acceleration")
        grid on
    
        % [max_accel, max_accel_idx] = max(abs(a_interp));
        % fprintf("Peak acceleration: %0.2f @ %0.2fs\n", max_accel, t(max_accel_idx));
        
        % hold on
        % plot(t,interp,'k')
        % plot(t,v_interp,'r--')
        % plot(t,a_interp,'b--')
        % title("Theta " + joint)
        % xlabel("time")
        % ylabel("theta")
        
    
        % % Plot via points
        % for via=1:k+1
        %     plot(T(via), theta(via),'ro')
        % end
        % legend("pos","vel","acc",'')
        sgtitle("Quintic interpolation between via points")
        
    end
    end