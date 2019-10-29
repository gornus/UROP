function [arr] = intersection(shp1, shp2, variables, arr)
% find the real intersection points between two shapes
    [m1,n1] = size(shp1);
    [m2,n2] = size(shp2);
    for io = 1:n1;
        for jo = 1:n2;
            [ix, iy] = solve([shp1(1,io), shp2(1,jo)], variables);
            o1 = size(ix);
            if o1>0
                for ko = 1:o1
%                     check1 = isreal([ix(ko) iy(ko)])
                    if isreal([ix(ko) iy(ko)])
%                         check2 = inrange([shp1(2,io) shp1(3,io)], [shp1(4,io) shp1(5,io)], [ix(ko) iy(ko)])
                        if inrange([shp1(2,io) shp1(3,io)], [shp1(4,io) shp1(5,io)], [ix(ko) iy(ko)]);
%                             check3 = inrange([shp2(2,jo) shp2(3,jo)], [shp2(4,jo) shp2(5,jo)], [ix(ko) iy(ko)])
                            if inrange([shp2(2,jo) shp2(3,jo)], [shp2(4,jo) shp2(5,jo)], [ix(ko) iy(ko)])
                                inter = [(ix(ko)); (iy(ko)); shp1(7,io); shp2(7,jo)];
                                arr =[arr, inter];
                            end
                        end
                    end
                end      
            end
        end
    end
end