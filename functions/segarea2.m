function [area] = segarea2(pt1, pt2, centre1, radius1, centre2, radius2)
% calculation of the area of the segment (bounded by a chord and an arc)
    ang1 = (atan(abs(pt1(2)-centre1(2)), abs(pt1(1)-centre1(1)))); % determine the angle of the intersections in respect to the centre of the arc bounding the segment
    ang2 = (atan(abs(pt2(2)-centre1(2)), abs(pt2(1)-centre1(1))));
    ang = ang2+ang1;
    polyarea = abs(0.5*sin(ang)*radius1^2);
    v1 = [pt1(1)-centre2(1) pt1(2)-centre2(2)]; % the vector between the intersection point and the centre of the other circle
    v2 = [centre1(1)-centre2(1) centre1(2)-centre2(2)]; % the vector between the centres
    v1p = (v1(1)*v2(1)+v1(2)*v2(2))/sqrt(v2(1)^2+v2(2)^2); % scalar projection to determine whether to use the reflex angle
    if v1p > norm(v2) % if using reflex angle --> sector + triangle
        ang = 2*sym(pi)-ang;
        secarea = ((ang)*radius1^2)/2;
        area = secarea + polyarea;
    else % otherwise --> sector - triangle
        secarea = ((ang)*radius1^2)/2;
        area = secarea - polyarea;
    end
end