function points = semicircle5(startP, endP, theta, numPoints)
%Return a linear sampled points of a semi-circle
% Args
% startp and endp : define points on diameter
% numpoints : number of samples

% Returns
% points : numPoints point sampled equally along arc starting at startp,
% with an angle of theta

% Use startp and endp to determine radius and centre 
start2D = startP(1:2);
end2D = endP(1:2);
z = startP(3);
diameter = end2D-start2D;
radius = norm(diameter/2);
centre = (start2D+end2D)/2;

% Get angles and interpolate
startCircle = start2D - centre;
phi = atan2(startCircle(2),startCircle(1)); % Starting position angle
angles = linspace(phi, phi + theta,numPoints);

% Compute points with this angle (rcos,rsin)
points = zeros(numPoints,3);
for i=1:size(angles,2)
    points(i,1) = radius*cos(angles(i));
    points(i,2) = radius*sin(angles(i));
    points(i,3) = z;
end