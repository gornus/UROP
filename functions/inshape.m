function [bool4] = inshape(point, shp)
% function to check whether the point is within the shape that is formed by
% multiple arcs
    [m1,n1] = size(shp);
    pt_lst = [];
    io = 1;
    bool4 = 0;
    % check whether point is in the segments
    while io<=n1 && bool4 == 0
        bool4 = insegment(point, [shp(2,io) shp(3,io)], [shp(4,io) shp(5,io)], shp(6,io));
        io = io+1;
    end
    if bool4 == 0
        for jo = 1:n1
            pt1 = [shp(4,jo) + shp(6,jo)*cos(shp(2,jo)); shp(5,jo) + shp(6,jo)*sin(shp(2,jo))];
            pt2 = [shp(4,jo) + shp(6,jo)*cos(shp(3,jo)); shp(5,jo) + shp(6,jo)*sin(shp(3,jo))];
            pt_lst = [pt_lst, pt1, pt2];
        end
        sort_pt = rankp(pt_lst);
        bool4 = inpolygon(point(1), point(2), sort_pt(1,:), sort_pt(2,:));
    else 
        bool4 = 1;
    end
end

