function [bool2] = insector(point, range, centre, radius)
% function to check whether the point is within the sector
    distance = sqrt((point(1)-centre(1))^2 + (point(2)-centre(2))^2);
    if distance < radius + 0.001 && inrange(range+0.001,centre, point)
        bool2 = 1;
    else
        bool2 = 0;
    end 
end