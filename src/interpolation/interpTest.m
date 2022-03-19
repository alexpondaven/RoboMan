% t1=1;
% t2=1;
% b3=2;
% b2=3;
% b1=0;
% 
% % coeffs = [a12,a13,a21,a22,a23]
% 
% A=[t1^2 t1^3 0 0 0
%    2*t1 3*t1^2 -1 0 0
%    0 0 t2 t2^2 t2^3
%    0 0 1 2*t2 3*t2^2
%    2 6*t1 0 -2 0];
% 
% b = [b2-b1
%      0
%      b3-b2
%      0
%      0];
% 
% linsolve(A,b)

% vias = [0 0 0 0
%         1 4 1 1
%         2 2 2 2];

%% Testing for toy case
% vias = [0 0 0 0
%         1 4 9 1
%         2 5 2 2
%         8 2 4 6];
% Tend =15;



%% Testing for square case
% z=90;
% % Corners
% square = [ 175 -50 z
%            175  50 z
%            75   50 z
%            75  -50 z ];
% 
% square(5,:) = square(1,:);  % make it complete the square
% 
% % interpolate lines between corners
% corners = [];
% numPoints = 10;
% for i=2:length(square)
%     corners = [corners; linearInterpolate(square(i-1,:), square(i,:), numPoints) ];
% end
% 
% vias = [];
% for j=1:size(corners, 1)
%     % corners(j,:)
%     theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2, GRIP_POS);
%     GRIP_ANGLE = theta(5);
%     vias = [vias; theta(1:4)];
% end
% Tend = 15;
% 
% vias % For debugging

% vias = [[0:10]' [0:10]' [0:10]' [0:10]'];

%% Cubic Interpolation
% [coeffs, T] = interpCubicTraj(vias,Tend)
% 
% plotCubicInterp(vias, coeffs, T)

%% Quintic Interpolation
% T = assignViaTimes(vias,Tend,'acc');
% coeffs = interpQuinticTraj(vias, T)
% 
% plotQuinticInterp(vias, coeffs, T)

%% Cell array
% vias = {[0 0 0 0
%         1 4 9 1
%         2 5 2 2
%         8 2 4 6]};


% vias = [0 0 0 0
%         1 4 9 1
%         2 8 2 2
%         2 12 2 2
%         2 16 2 2
%         2 20 2 2
%         8 24 4 6];


startPos = inverseKinDynamixel2(225, 0, 50, 0, true);
endPos = inverseKinDynamixel2(225, 0, 25, 0, true);
vias = linearInterpolate(startPos(1:4), endPos(1:4), 10);

occupancyGrid = createOccupancyGrid([0,1,0,0,0,0]);

startPos = [225, 0, 50, 0];
endPos = [100, 100, 50, -pi/2];
vias = calcViaPoints(startPos, endPos, occupancyGrid);


Tend =15;

%% Plot different interpolation strategies on the same plot
figure

[T_lin, Tend_lin] = assignViaTimes(vias, 'lin');    % Tend no longer used
coeffs_lin = interpQuinticTraj(vias, T_lin);
[T_dpos, Tend_dpos] = assignViaTimes(vias, 'dpos');    % Tend no longer used
coeffs_dpos = interpQuinticTraj(vias, T_dpos);
[T_acc, Tend_acc] = assignViaTimes(vias, 'acc');    % Tend no longer used
coeffs_acc = interpQuinticTraj(vias, T_acc);
[T_dvel, Tend_dvel] = assignViaTimes(vias, 'dvel');    % Tend no longer used
coeffs_dvel = interpQuinticTraj(vias, T_dvel);

dT_lin = diff(T_lin);
dT_dpos = diff(T_dpos);
dT_acc = diff(T_acc);
dT_dvel = diff(T_dvel);

k = size(dT_lin,2);
for joint=1:4
    theta = vias(:,joint);
    % subplot(2,2,joint)
    dt = 0.01;
    t_lin = [];
    t_dpos = [];
    t_acc = [];
    t_dvel = [];
    
    interp_lin = [];
    v_interp_lin = [];
    a_interp_lin = [];
    interp_dpos = [];
    v_interp_dpos = [];
    a_interp_dpos = [];
    interp_acc = [];
    v_interp_acc = [];
    a_interp_acc = [];
    interp_dvel = [];
    v_interp_dvel = [];
    a_interp_dvel = [];

    for j=1:k
        i = 6*(j-1) + 1;
        jt = 0:dt:dT_lin(j);
        jInterp_lin = coeffs_lin(i,joint) + coeffs_lin(i+1,joint)*jt + coeffs_lin(i+2,joint)*jt.^2 + coeffs_lin(i+3,joint)*jt.^3 + coeffs_lin(i+4,joint)*jt.^4 + coeffs_lin(i+5,joint)*jt.^5;
        vInterp_lin = coeffs_lin(i+1,joint) + 2*coeffs_lin(i+2,joint)*jt + 3*coeffs_lin(i+3,joint)*jt.^2 + 4*coeffs_lin(i+4,joint)*jt.^3 + 5*coeffs_lin(i+5,joint)*jt.^4;
        aInterp_lin = 2*coeffs_lin(i+2,joint) + 6*coeffs_lin(i+3,joint)*jt + 12*coeffs_lin(i+4,joint)*jt.^2 + 20*coeffs_lin(i+5,joint)*jt.^3;
        jt = jt + T_lin(j);
        
        t_lin = [t_lin jt];
        interp_lin = [interp_lin jInterp_lin];
        v_interp_lin = [v_interp_lin vInterp_lin];
        a_interp_lin = [a_interp_lin aInterp_lin];
    end

    for j=1:k
        i = 6*(j-1) + 1;
        jt = 0:dt:dT_dpos(j);
        jInterp_dpos = coeffs_dpos(i,joint) + coeffs_dpos(i+1,joint)*jt + coeffs_dpos(i+2,joint)*jt.^2 + coeffs_dpos(i+3,joint)*jt.^3 + coeffs_dpos(i+4,joint)*jt.^4 + coeffs_dpos(i+5,joint)*jt.^5;
        vInterp_dpos = coeffs_dpos(i+1,joint) + 2*coeffs_dpos(i+2,joint)*jt + 3*coeffs_dpos(i+3,joint)*jt.^2 + 4*coeffs_dpos(i+4,joint)*jt.^3 + 5*coeffs_dpos(i+5,joint)*jt.^4;
        aInterp_dpos = 2*coeffs_dpos(i+2,joint) + 6*coeffs_dpos(i+3,joint)*jt + 12*coeffs_dpos(i+4,joint)*jt.^2 + 20*coeffs_dpos(i+5,joint)*jt.^3;
        jt = jt + T_dpos(j);
        
        t_dpos = [t_dpos jt];
        interp_dpos = [interp_dpos jInterp_dpos];
        v_interp_dpos = [v_interp_dpos vInterp_dpos];
        a_interp_dpos = [a_interp_dpos aInterp_dpos];
    end
    
    for j=1:k
        i = 6*(j-1) + 1;
        jt = 0:dt:dT_acc(j);
        jInterp_acc = coeffs_acc(i,joint) + coeffs_acc(i+1,joint)*jt + coeffs_acc(i+2,joint)*jt.^2 + coeffs_acc(i+3,joint)*jt.^3 + coeffs_acc(i+4,joint)*jt.^4 + coeffs_acc(i+5,joint)*jt.^5;
        vInterp_acc = coeffs_acc(i+1,joint) + 2*coeffs_acc(i+2,joint)*jt + 3*coeffs_acc(i+3,joint)*jt.^2 + 4*coeffs_acc(i+4,joint)*jt.^3 + 5*coeffs_acc(i+5,joint)*jt.^4;
        aInterp_acc = 2*coeffs_acc(i+2,joint) + 6*coeffs_acc(i+3,joint)*jt + 12*coeffs_acc(i+4,joint)*jt.^2 + 20*coeffs_acc(i+5,joint)*jt.^3;
        jt = jt + T_acc(j);
        
        t_acc = [t_acc jt];
        interp_acc = [interp_acc jInterp_acc];
        v_interp_acc = [v_interp_acc vInterp_acc];
        a_interp_acc = [a_interp_acc aInterp_acc];
    end

    for j=1:k
        i = 6*(j-1) + 1;
        jt = 0:dt:dT_dvel(j);
        jInterp_dvel = coeffs_dvel(i,joint) + coeffs_dvel(i+1,joint)*jt + coeffs_dvel(i+2,joint)*jt.^2 + coeffs_dvel(i+3,joint)*jt.^3 + coeffs_dvel(i+4,joint)*jt.^4 + coeffs_dvel(i+5,joint)*jt.^5;
        vInterp_dvel = coeffs_dvel(i+1,joint) + 2*coeffs_dvel(i+2,joint)*jt + 3*coeffs_dvel(i+3,joint)*jt.^2 + 4*coeffs_dvel(i+4,joint)*jt.^3 + 5*coeffs_dvel(i+5,joint)*jt.^4;
        aInterp_dvel = 2*coeffs_dvel(i+2,joint) + 6*coeffs_dvel(i+3,joint)*jt + 12*coeffs_dvel(i+4,joint)*jt.^2 + 20*coeffs_dvel(i+5,joint)*jt.^3;
        jt = jt + T_dvel(j);

        t_dvel = [t_dvel jt];
        interp_dvel = [interp_dvel jInterp_dvel];
        v_interp_dvel = [v_interp_dvel vInterp_dvel];
        a_interp_dvel = [a_interp_dvel aInterp_dvel];
    end
    
    % Plot lines
    subplot(4,3, 3*(joint-1)+1)     % position
    plot(t_lin, interp_lin, 'r-')
    hold on
    plot(t_dpos, interp_dpos, 'g-')
    plot(t_acc, interp_acc, 'b-')
    plot(t_dvel, interp_dvel, 'k-')
    title("Theta " + joint + " position")
    for via=1:k+1
        plot(T_lin(via), theta(via),'ro')
        plot(T_dpos(via), theta(via),'go')
        plot(T_acc(via), theta(via),'bo')
        plot(T_dvel(via), theta(via),'ko')
    end
    legend("lin", "dpos", "acc", "dvel")
    grid on
    
    subplot(4,3, 3*(joint-1)+2)     % velocity
    plot(t_lin, v_interp_lin, 'r-')
    hold on
    plot(t_dpos, v_interp_dpos, 'g-')
    plot(t_acc, v_interp_acc, 'b-')
    plot(t_dvel, v_interp_dvel, 'k-')
    title("Theta " + joint + " velocity")
    legend("lin", "dpos", "acc", "dvel")
    grid on
    
    subplot(4,3, 3*(joint-1)+3)     % acceleration
    plot(t_lin, a_interp_lin, 'r-')
    hold on
    plot(t_dpos, a_interp_dpos, 'g-')
    plot(t_acc, a_interp_acc, 'b-')
    plot(t_dvel, a_interp_dvel, 'k-')
    title("Theta " + joint + " acceleration")
    legend("lin", "dpos", "acc", "dvel")
    grid on
    
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



% interpViaPoints(vias,true)