function [range] = rangecorrection(range)
%make sure the angles are correct, ie. the ending angle is always larger
%than the starting angle, but does go too far
%   Detailed explanation goes here
Pi = sym(pi);
if range(2) < range(1)
        range(2) = range(2) + 2*Pi;
    end
    if range(2) > range(1) + 2*Pi
        range(2) = range(2) - 2*Pi;
    end
end