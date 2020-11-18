function [optWallWaypointTruth,wallToCheck,waypointPair] = optWallWaypointCheck(waypointToCheck,waypointsOptWall4,optWalls)
%OPTWALLWAYPOINTCHECK check if waypoint is an opt wall waypoint. If it is,
%also return its pair
%
%
%INPUTS
%   waypointToCheck = waypoint to check
%   waypointsOptWall4 = list of all the optional wall waypoints. nx4. Each
%       row corresponds to an optional wall, cols 1 and 2 correspond to
%       waypoint 1 and cols 3 and 4 correspond to waypoint 2
%   optWalls = list of the optional walls
%
%OUTPUTS
%   truth = logical of if waypointsToCheck is an optional wall waypoint
%   wallToCheck = which wall waypoint corresponds to. Which wall am I
%       checking now. 1x4
%   waypointPair =
%       if truth = 1, then this returns the waypoints' pair, if available
%       if truth = 0 or if pair doesn't exist, then this is [inf inf]
%
%

optWallWaypointTruth = false;
waypointPair = [inf inf];

waypoints1 = waypointsOptWall4(:,1:2);
waypoints2 = waypointsOptWall4(:,3:4);

rowLogical1 = all(waypoints1 == waypointToCheck,2);
rowLogical2 = all(waypoints2 == waypointToCheck,2);

rowContainingWaypoint = any([rowLogical1 rowLogical2],2);

if any(rowContainingWaypoint)
    %Now find its pair
    if isequal(rowContainingWaypoint,rowLogical1)
        waypointPair = waypoints2(rowLogical1,:); 
    else
        waypointPair = waypoints1(rowLogical2,:);
    end
    
    %If there is a pair
    if ~any(isinf(waypointPair))
        optWallWaypointTruth = true;
    end
end

%Find which wallToCheck this corresponds to.
rowLogical = rowLogical1 | rowLogical2;
wallToCheck = optWalls(rowLogical,:);

end


