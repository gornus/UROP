function [result] = larger(Circle1,Circle2)
% determine which circle is larger
%   Detailed explanation goes here
if Circle1(6) >= Circle2(6)
    result = Circle1;
else
    result = Circle2;
end

