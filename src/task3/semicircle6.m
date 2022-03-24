function points = semicircle6(startP, center, theta, numPoints)
%Return a linear sampled points of a semi-circle
% Args
% startp    : start point
% center    : center coordinates of circle
% endTheta  : the theta angle of the arc to draw. 
% Positive is anticlockwise, negative is clockwise
% numPoints : number of points to sample
%
% Returns
% points : numPoints point sampled equally along arc starting at startp,
% with an angle of theta

    start2D = startP(1:2);
    center2D = center(1:2);
    z = startP(3);
    radVect = start2D-center2D;
    radius = norm( radVect );

    % Get angles and interpolate
    phi = atan2( radVect(2), radVect(1) );
    angles = linspace(phi, phi+theta, numPoints);
    fprintf("Start: %0.1f . End: %0.1f\n", rad2deg(phi), rad2deg(angles(end)));


    % Compute points with this angle (rcos,rsin) + offset by centre
    points = zeros(numPoints,4);
    for i=1:size(angles,2)
        points(i,1) = radius*cos(angles(i)) + center(1);
        points(i,2) = radius*sin(angles(i)) + center(2);
        points(i,3) = z;
    end

    figure
    plot(points(:,1), points(:,2))
%     axis([-1 1 -1 1])
    grid on

end