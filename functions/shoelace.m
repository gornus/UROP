function [area] = shoelace(points) 
% calculate the area of a polygon by only knowing the corners
    area = 0;
    len = size(points,2);
    for i = 0:len-1
        area = area + points(1,mod(i,len)+1)*points(2,mod(i+1,len)+1) - points(1,mod(i+1,len)+1)*points(2,mod(i,len)+1);
    end
    area = abs(area)/2;
end