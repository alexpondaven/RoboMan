function points = semicircle2(center, radius, numPoints)
%Return a linear sampled poitns of a semi-circle
%Args
%center   center of the circle
%radius   radius of the circle
%numpoints     number of samples between points
points = zeros(numPoints, length(center));
%arc-length is determined by angle
arc = pi;
delta = arc/(numPoints-1);

    for i=0:numPoints-1
        if i < arc/2
         points(i+1,1)= center(1) - cos(delta*i)*radius;
         %positive and negative depends on upper/lower semicircle
         points(i+1,2) =center(2) + sin(delta*i)*radius;
         points(i+1,3) = center(3);
         %fprintf("%d\n", 1);
        elseif i == arc/2
         fprintf("%d\n", 1);
         points(i+1,1)= center(1);
         %positive and negative depends on upper/lower semicircle
         points(i+1,2) =center(2)+radius;
         points(i+1,3) = center(3);
        else
         points(i+1,1)= center(1) - cos(delta*i)*radius;
         %positive and negative depends on upper/lower semicircle
         points(i+1,2) =center(2) + sin(delta*i)*radius;
         points(i+1,3) = center(3);
        end
            

%positive and negative depends on upper/lower semicircle

    end


end