function [output] = rankp(input)
% rank the points in counterclockwise order
    %corner = double(corner);
    lenc = size(input,2);
    % computing the average position of the points
    ax = 0;
    ay = 0;
    for points = input;
        ax = ax + points(1);
        ay = ay + points(2);
    end
    ax = ax/lenc;
    ay = ay/lenc;
    % finding the relative angle of each point
    angle = [];
    for points = input;
        ang = angle2(points,[ax;ay]);
        angle = [angle ang];
    end
    % sorting the points in anti-clockwise order
    input = [input; angle];
    [m1,n1] = size(input);
    [temp, order] = sort(input(m1,:));
    sortp = input(:,order);
    output = sortp(1:m1-1,:);
end