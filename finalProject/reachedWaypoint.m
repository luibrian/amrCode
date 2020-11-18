function [dataStore,newPose,mapStruct,waypointStruct,RRTparam] = reachedWaypoint(dataStore,CreatePort,robPose,waypointToCheck,mapStruct,waypointStruct,covStruct,miscStruct,RRTparam)
%REACHEDWAYPOINT You've reached a waypoint. Now what?
%1. Figure out which type of waypoint you are on. 
%2. Do unique things based on this type of waypoint
%3. Update the map, waypoints, RRT, after doing your thing
%
%INPUTS:
%   CreatePort = so I can turn
%   robPose = what the current robot pose is
%   waypointToCheck = current waypoint the robot thinks it's on. goal
%   mapStruct = map structure containing map info
%   waypointStruct = struct containing waypoint info
%   covStruct = struct containing covariance info
%   miscStruct = struct containing other useful info
%   RRTparam = struct containing RRT info
%   
%OUTPUTS:
%   newPose = the current robot pose
%   mapStruct = upvdated map info
%   waypointStruct= updated waypoint info


%Extracting some information out
waypointsOptWall4 = waypointStruct.waypointsOptWall4;
optWalls = mapStruct.optWalls;

%Initializing some variables
newPose = robPose;
wallTruth = 1;

%Check whether waypoint is an opt wall waypoint
[optWallWaypointTruth,wallToCheck,waypointPair] = optWallWaypointCheck(waypointToCheck,waypointsOptWall4,optWalls);

%Do things specific to which waypoint it is on
if optWallWaypointTruth %I'm currently on an optional wall waypoint
    %Figure out how much to turn
    [angleToTurn,robPoseTurned] = optWallWaypointTurn(robPose,wallToCheck);
    newPose = robPoseTurned;
    
    turnAngle(CreatePort, 0.2, angleToTurn);

    currentSize = length(dataStore.rsdepth(:,1));
   
    
    %depthData = 0;
    
    
    wallTruth =  wallCheckSequence(robotPose,depthData,miscStruct,mapStruct,covStruct);
    %Definitely missing some inputs
    
else
    actualWaypointsSequence(CreatePort); 
end

%IF I WANT TO MAKE THIS CLEAN, SHOULD SPLIT THE ABOVE FUNCTION UP AND TOSS
%IN ABOVE IF STATEMENT
wallChecked = wallToCheck;
[mapStruct,waypointStruct] = updateMapWaypoints(waypointToCheck,optWallWaypointTruth,waypointPair,wallChecked,wallTruth,mapStruct,waypointStruct);


%% Generate a new RRT
mapEight = mapStruct.mapEight;
mapBounds = mapStruct.mapBounds;
R= miscStruct.R;
initPoint = newPose;
goalPoints = waypointStruct.totalWaypoints;

[RRT,path,goal] = buildRRT(mapEight,mapBounds,R,initPoint,goalPoints,RRTparam);
RRTparam.RRT = RRT;
RRTparam.path = path;
RRTparam.goal = goal;


end

