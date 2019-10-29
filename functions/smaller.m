function [result] = smaller(Circle1,Circle2)
% determine which circle is smaller
%   Detailed explanation goes here
if Circle1(6) >= Circle2(6)
    result = Circle2;
else
    result = Circle1;
end

