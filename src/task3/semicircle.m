function points = semicircle(center, radius, numPoints)
%Return a linear sampled poitns of a semi-circle
%Args
%center   center of the circle
%radius   radius of the circle
%numpoints     number of samples between points
points = zeros(numPoints, length(center));

delta = (2*radius)/(numPoints-1);

    for i=0:numPoints-1
         points(i+1,1)= (center(1)-radius)+ delta*i;
         %positive and negative depends on upper/lower semicircle
         points(i+1,2) =center(2)+sqrt(radius^2-((center(1)-radius)+ delta*i-center(1))^2);
         points(i+1,3) = center(3);
         %fprintf("%d\n", 1);

%positive and negative depends on upper/lower semicircle

    end


end





