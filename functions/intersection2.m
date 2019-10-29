function [arr] = intersection2(shp1, shp2, variables, arr)
% find the real intersection points between two shapes
[ix, iy] = solve([shp1(1), shp2(1)], variables);
o1 = size(ix,1);
if o1>0
    for ko = 1:o1
        if isreal([ix(ko) iy(ko)])
            inter = [ix(ko); iy(ko)];
            arr =[arr, inter];
        end                            
    end      
end

end