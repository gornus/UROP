function [area] = segarea(pt1, pt2, centre, radius)
%     polyarea = abs(shoelace([pt1 pt2 centre]'));
    ang1 = (atan(abs(pt1(2)-centre(2)), abs(pt1(1)-centre(1))))
    ang2 = (atan(abs(pt2(2)-centre(2)), abs(pt2(1)-centre(1))))
    ang = ang2+ang1;
    polyarea = abs(0.5*sin(ang)*radius^2);
    secarea = ((ang)*radius^2)/2;
    area = secarea - polyarea;
end