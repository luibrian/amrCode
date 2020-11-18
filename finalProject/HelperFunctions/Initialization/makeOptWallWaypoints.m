function [waypointsArray4,toCheckWaypoints] = makeOptWallWaypoints(optWall,mapFour,mapBounds,R,wallThickness,buffer,Vmat,Emat)
%OPTWALLWAYPOINTS Puts two waypoints on either side of the opt wall
% 
%INPUTS
%   optWall- nx4 of the optional walls to check. n x [x1 y1 x2 y2]
%   mapFour- nx4 containing the map obstacles
%   mapBounds = [xmin ymin xmax ymax] of the map boundaries
%   R = robot radius
%   wallThickness = thickness of the walls
%   buffer = starting additional buffer
%   
%
%OUTPUTS
%   waypointsArray- max (2n)x2 containing the [x y] for each waypoint. 2 per
%   toCheckWaypoints- nx2 array of waypoints that toss out all the inf in
%       waypoints

%Removing the boundaries of the map

numOptWalls = length(optWall(:,1));
waypointsArray4 = inf(numOptWalls,4);
%hasPair = zeros(numOptWalls,1);
for i = 1:length(optWall(:,1)) %do for every wall
    
    wall = optWall(i,:); %nx4
    
    % Finding which direction to put the two waypoints
    wallMidpoint = [mean([wall(1),wall(3)]), mean([wall(2),wall(4)])];
    point = [wall(3) wall(4)];
    alongWallNormVector = (point - wallMidpoint)/ norm(point-wallMidpoint);
    normalToWallVector = cross([0 0 1], [alongWallNormVector,0]);
    normalToWallVector = normalToWallVector(1:2);
    
    
    suc1 = 0;
    suc2 = 0;
    currentBuffer = buffer;
    while (currentBuffer >= 0.1) && (~suc1 || ~suc2)
        distToWall = R+ wallThickness/2 + currentBuffer;
        
        if ~suc1
            point = wallMidpoint + normalToWallVector * distToWall;
            intersectObsTruth = intersectObstacleTruth(point,wallMidpoint,Vmat,Emat);
            if ~intersectObsTruth
                waypointsArray4(i,1:2) = point;
                suc1 = true;
            end
        end
        
        if ~suc2
            point = wallMidpoint - normalToWallVector * distToWall;
            intersectObsTruth = intersectObstacleTruth(point,wallMidpoint,Vmat,Emat);
            if ~intersectObsTruth
                waypointsArray4(i,3:4) = point;
                suc2 = true;
            end
        end

        currentBuffer = currentBuffer - 0.1;
    end
    
    %if suc1 && suc2
    %    hasPair(i) = 1;
    %end
end

%Cleaning up waypoints
toCheckWaypoints = inf(numOptWalls*2,2);
%Filling in this
for i = 1:numOptWalls
    waypoint1 = waypointsArray4(i,1:2);
    waypoint2 = waypointsArray4(i,3:4);
    toCheckWaypoints(2*i-1,:) = waypoint1;
    toCheckWaypoints(2*i,:) = waypoint2;
end
toCheckWaypoints(any(isinf(toCheckWaypoints),2),:) = [];


noPairIndices = find(any(isinf(waypointsArray4),2));
for i = 1:length(noPairIndices)
    waypoint1 = waypointsArray4(noPairIndices(i),1:2);
    waypoint2 = waypointsArray4(noPairIndices(i),3:4);
    
    if any(isinf(waypoint1)) %moving all the good waypoints to the first two cells
        if ~any(isinf(waypoint2))
            waypointsArray4(noPairindices(i),1:2) = waypoints2;
        end
    end

%    if any(isinf(waypoint1)) && any(isinf(waypoint2))
        %This couldn't find a good point to check the optional wall
        
%    end
end




end



