function [mapStruct,waypointStruct] = updateMapWaypoints(waypointToCheck,optWallWaypointTruth,waypointPair,wallChecked,wallTruth,mapStruct,waypointStruct,miscStruct)
%UPDATEMAP After checking an optional wall, update the map and waypoints
%   
%INPUTS:
%   waypointToCheck = current waypoint the robot thinks its on
%   optWallWaypointTruth = logical of whether the current waypoint is an opt wall
%       waypoint
%   waypointPair = if optWallWaypointTruth is 1, then this is the
%       waypoint's pair. Otherwise, [inf inf].
%   wallChecked- which wall the robot just checked. [x1 y1 x2 y2]
%   wallTruth = is wallChecked an actual wall 
%   mapStruct = struct that contains the map info
%   waypointStruct
%       waypoints = waypoints that we start with
%       ECwaypoints = EC waypoints that we start with
%       waypointsOptWall4 = waypoints that correspond to optWalls
%       totalWaypoints = waypoints that still need to be visited
%       vistedWaypoints = waypoints visited
%       
%   waypoints = total waypoints that still need to be visited
%   waypointsOptWall4- waypoints that correspond to optWalls


%Extracting information out
mapFour = mapStruct.mapFour;
optWalls = mapStruct.optWalls;
totalWaypoints = waypointStruct.totalWaypoints;
visitedWaypoints = waypointStruct.visitedWaypoints;
wallThickness = miscStruct.wallThickness;

%% Updating waypoints
totalWaypoints(all(totalWaypoints == waypointToCheck,2),:) = [];
if optWallWaypointTruth
    %Pop off the extra waypoint if waypoint is from optWall
    totalWaypoints(all(totalWaypoints == waypointPair,2),:) = [];
else 
    %Updating the visited waypoints only if not an optWall waypoint
    visitedWaypoints = [visitedWaypoints; waypointToCheck];
end

%% Updating the map
%Pop off the opt wall that was checked 

%Update map if it's an optional wall waypoint
if optWallWaypointTruth  
    optWalls(all(optWalls==wallChecked,2),:) = [];
    
    if ~wallTruth %if wall is not there
        mapFour(any(mapFour == wallChecked,2),:) = [];

        %Generate a new mapEight
        [xMin, xMax, yMin, yMax] = minMaxMap(mapFour);
        mapBounds = [xMin yMin xMax yMax];
        mapEight = mapFormatChange(mapFour,mapBounds,wallThickness);
        mapStruct.mapEight = mapEight;
    end
end

%% Putting things back into the structure
waypointStruct.totalWaypoints = totalWaypoints;
waypointStruct.visitedWaypoints = visitedWaypoints;
mapStruct.optWalls = optWalls;
mapStruct.mapFour = mapFour;

end

