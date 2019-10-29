function [bool3] = insegment(point, range, centre, radius)
% function to check whether the point is within the segment
    startp = [centre(1)+radius*sin(range(1));centre(2)+radius*cos(range(1))]; % convert the stard and end angle to points
    endp = [centre(1)+radius*sin(range(2));centre(2)+radius*cos(range(2))];
    xv = [centre(1) startp(1) endp(1)];
    yv = [centre(2) startp(2) endp(2)];
    if insector(point, range, centre, radius)
        if inpolygon(point(1),point(2), xv, yv);
            bool3 = 0;
        else
            bool3 = 1;
        end
    else
        bool3 = 0;
    end
end