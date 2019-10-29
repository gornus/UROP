function [result] = smaller(Circle1,Circle2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if Circle1(6) >= Circle2(6)
    result = Circle2;
else
    result = Circle1;
end

