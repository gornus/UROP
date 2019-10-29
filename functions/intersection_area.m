function [area] = intersection_area(Circle1,Circle2)
%Calculate the intersecting area between the two circles
%   Calculate the intersecting area between the two circles
    syms x y
    Pi = sym(pi);
    area = 0;
    intersections = [];
    % Shape2 = [Cir2;0;2*Pi;O2(1);O2(2);R2;2]; <- data structure
    intersections = intersection2(Circle1,Circle2,[x y],intersections);
    [m,n] = size(intersections);
    centre1 = [Circle1(4) Circle1(5)];
    centre2 = [Circle2(4) Circle2(5)];
    if n > 1
        point1 = intersections(:,1);
        point2 = intersections(:,2);
        temp1 = vpa(segarea2(point1, point2,centre1,Circle1(6),centre2,Circle2(6)));
        area = area + temp1;
        temp2 = vpa(segarea2(point1, point2,centre2,Circle2(6),centre1,Circle1(6)));
        area = area + temp2;
    else
        area = 0;
    end
    area;
end

