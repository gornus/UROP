function [bool] = inrange(range,centre,point)
% function to check whether the point is within the range of the arc
    if range(1) > range(2)
        range(2) = range(2) + 2*pi;
    end
    ang = angle2(point,centre);
    if ang < range(1)
        ang = ang + 2*pi;
    end
    if ang > range(1) && ang < range(2);
        bool = 1;
    else
        bool = 0;
    end
end