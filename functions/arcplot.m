%% plotting functions
function h = arcplot(range, centre, radius, style)  
%     double(range)
%     double(rad2deg(range))
    plotrang = linspace(range(1), range(2));
%     double(rad2deg(plotrang))
    hold on
    xunit = radius * cos(plotrang) + centre(1);
    yunit = radius * sin(plotrang) + centre(2);
    double(xunit);
    double(yunit);
    h = plot(xunit, yunit,style);
end