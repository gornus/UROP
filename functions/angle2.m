function [outang] = angle2(point1, point2) 
% calculate the angle between two points and the x-axis, correct to be always positive
    outang = atan2(point1(2)-point2(2),point1(1)-point2(1));
    if outang < 0
        outang = 2*pi + outang;
    end
end