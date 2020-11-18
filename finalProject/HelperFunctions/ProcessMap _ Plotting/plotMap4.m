function plotMap4(map,n)
    %Plots the map. 
    %map is a 4xn array in which each row represents a line segment
    %The first column is the x coord of the first point, the second column
    %is the y coord of the first point
    %The third column is the x coord of the second point, the fourth column
    %is the y coord of the second point
    %n is the figure number
    
    point1X = map(:,1);
    point1Y = map(:,2);
    point2X = map(:,3);
    point2Y = map(:,4);
    
    figure(n);
    for i=1:length(point1X)
        lineSegX = [point1X(i), point2X(i)];
        lineSegY = [point1Y(i), point2Y(i)];
        hold on;
        plot(lineSegX,lineSegY,'k', 'HandleVisibility','off');
        hold on;
        plot(point2X(i),point2Y(i),'k','HandleVisibility','off');
        
        plot(inf, inf,'HandleVisibility','off');
    end