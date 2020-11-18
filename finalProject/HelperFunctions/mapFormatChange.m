function [obstacleMap] = mapFormatChange(wallmap,mapBounds,thickness)
%MAPFORMATCHANGE 
%   Takes in a map that's gives boundary endpoints and converts it into
%   nx16 format in which each line is a boundary
%
%   INPUTS:
%       wallmap- nx4. 4 of these are the boundary walls, we don't want
%       those
%       mapBounds = [xmin ymin xmax ymax];
%       thickness to give the walls
%
%   OUTPUTS
%       obstacleMap = nx8

numWalls = length(wallmap(:,1));

obstacleMap = zeros(numWalls-4,8);
j = 1;
for i = 1:numWalls
    boundaryWall = ismember(wallmap(i,1),mapBounds) && ...
        ismember(wallmap(i,2),mapBounds) && ...
        ismember(wallmap(i,3),mapBounds) && ...
        ismember(wallmap(i,4),mapBounds);
    
    if ~boundaryWall
    
        lineVector = wallmap(i,1:2)- wallmap(i,3:4);
        alongLineDir = lineVector / norm(lineVector);

        %Getting the normal direction
        normLineDir = cross([0 0 1], [alongLineDir 0]);
        normLineDir = normLineDir(1:2);

        %Getting the components
        point1 = wallmap(i,1:2) + thickness/2 * normLineDir;
        point2 = wallmap(i,1:2) - thickness/2 * normLineDir;
        point3 = wallmap(i,3:4) - thickness/2 * normLineDir;
        point4 = wallmap(i,3:4) + thickness/2 * normLineDir;
        obstacleMap(j,1:8) = [point1, point2, point3, point4];
        j = j+1;
    end
end

%obstacleMap = obstacleMap(:,1:8);


end

