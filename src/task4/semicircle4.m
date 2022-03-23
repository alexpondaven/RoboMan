function points = semicircle4(startp, endp, theta, numPoints)
%Return a linear sampled poitns of a semi-circle
%Args
%center   center of the circle
%radius   radius of the circle
%numpoints     number of samples between points
points = zeros(numPoints, length(startp));
temp = zeros(numPoints, length(startp));

%arc-length is determined by angle
arc = theta;
delta = arc/(numPoints-1);
radius = sqrt((startp(1)-endp(1))^2+(startp(2)-endp(2))^2);
center = (startp+endp)./2;
    for i=0:numPoints-1
        temp(i+1,1)= center(1) - cos(delta*i)*radius;
        %positive and negative depends on upper/lower semicircle
        temp(i+1,2) =center(2) + sin(delta*i)*radius;
        temp(i+1,3) = center(3);
        %fprintf("%d\n", 1);
    end
    points=[];    
    for i=0:numPoints-1
        if temp(i+1,1)<=0 && temp(i+1,2)>=0
            points=[points;temp(i+1,:)];
        else
            %points=[points;temp(i+1,:)];
        end
    end
   
                
            
            
end