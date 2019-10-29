function [bool1] = incircle(point, circle)
% function to check whether the point is within the circle
    distance = sqrt((point(1)-circle(4))^2 + (point(2)-circle(5))^2);
    circle(6);
    if eval(distance) <= circle(6)+0.00000000001;
        bool1 = 1;
    else
        bool1 = 0;
    end
end