function [xMin, xMax, yMin, yMax] = minMaxMap(map)
    %MINMAXMAP returns the max and min x and y values of the points in the map
    %map is an nx4 matrix in which the first column is the first point's x
    %coord, the second column is the first point's y coord, the third
    %column is the second point's x coord, the fourth column is the second
    %point's y coord. These two points define a line segment.
    %These points are in Cartesian coordinates
    
    maxMap = max(map);      %takes the max of each column
    minMap = min(map);      %takes the min of each column
    
    xMax = max(maxMap(1),maxMap(3));
    yMax = max(maxMap(2),maxMap(4));

    xMin = min(minMap(1),minMap(3));
    yMin = min(minMap(2),minMap(4));
end    