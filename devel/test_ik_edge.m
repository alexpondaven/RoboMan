% Test to check edge cases for the IK function (which sometimes seems to fail)

goal = [45, 0, 45, -pi/2];
[theta, ec] = inverseKin2(goal(1), goal(2), goal(3), goal(4), true);
rad2deg(theta)
IK_ErrorCodes(ec);

joints = getJointPositions(theta)

%% Plotting here
plot3([0,250], [0,  0], [0,  0], 'r', 'LineWidth', 1.0)
hold on
plot3([0,  0], [0,250], [0,  0], 'g', 'LineWidth', 1.0)
plot3([0,  0], [0,  0], [0,250], 'b', 'LineWidth', 1.0)
axis([-100, 250, -250, 250, 0, 250])

for visIdx=2:size(joints,2)
    drawLine(joints(:,visIdx-1), joints(:,visIdx));
    % drawFrame(joints(:,visIdx), 20);
end

axis([-50, 250, -250, 250, 0, 250])

plot3(goal(1), goal(2), goal(3), 'gx', 'LineWidth', 5)

grid on
hold off