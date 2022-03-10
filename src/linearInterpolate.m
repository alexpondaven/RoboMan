function points = linearInterpolate(point1, point2, numPoints)
% Returns a linearly interpolated list of points along two points in the xyz axis.
% 
% ARGS
% point1, point2    : Start and end points of line
% numPoints         : Number of points to sample between points

    points = zeros(numPoints, length(point1));

    delta = (point2-point1) / (numPoints-1);

    for i=0:numPoints-1
        points(i+1,:) = delta*i + point1;
    end

end

