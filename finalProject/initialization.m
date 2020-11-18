function [mapStruct,beaconLoc,waypointStruct,RRTparam,covStruct,miscStruct] = initialization(mapTxt,initWaypoint,sensorOrigin)
%INITIALIZATION Set up all the waypoints, maps, and a RRT
%
%INPUTS
%   mapTxt = text file that contains map information. Will be 'compMap.mat'
%   initWaypoint = starting waypoint of the robot
%   sensorOrigin = where sensor is relative to robot origin [x y]
%
%OUTPUTS:
%   mapStruct = map structure containing map information (obstacles and
%       optWalls and map boundaries)
%   beaconLoc = location of all the beacons
%   waypointStruct = contains waypoint information
%       waypoints = original set of waypoints
%       ECwaypoints = original set of EC waypoints
%       optWaypointsArray = optional wall waypoints. nx4
%       waypointsToCheck = all the waypoints needed to be checked
%   RRTparam = structure containing the parameters for the RRT
%   covStruct = structure containing covariance of noise
%   miscStruct = structure containing other miscellaneous but useful info

%% Some constants
wallThickness = 0.1;    %Wall thickness
R = 0.13;               %Robot radius
buffer =0.2;            %Used to make the opt wall waypoints. Starting size

%% Unloading the mapTxt information
[mapStruct,beaconLoc,waypointStruct] = processInput(mapTxt);
mapFour = mapStruct.mapFour;
mapEight = mapStruct.mapEight;
mapBounds = mapStruct.mapBounds;
optWalls = mapStruct.optWalls;
waypoints = waypointStruct.waypoints;
ECwaypoints = waypointStruct.ECwaypoints;


%% Creating the waypoints associated with optWalls
mapEightNoOptWalls = mapEight(1:end-length(optWalls(:,1)),:);
mapBloatedNoOptWalls = createBloatedMap(mapEightNoOptWalls,R);
[~, ~,Vmat,Emat] = obsGraph(mapBloatedNoOptWalls);

[optWaypointsArray4,toCheckWaypoints] = makeOptWallWaypoints(optWalls,mapFour,mapBounds,R,wallThickness,buffer,Vmat,Emat);
totalWaypoints = [waypoints; ECwaypoints; toCheckWaypoints];
totalWaypoints(any(totalWaypoints == initWaypoint,2),:) = []; %Remove current one from here

waypointStruct.optWaypointsArray4 = optWaypointsArray4;   %Waypoints for opt walls


waypointStruct.totalWaypoints = totalWaypoints;         %All waypoints to check
waypointStruct.visitedWaypoints = initWaypoint;


%% Goal points of RRT
goalPoints = totalWaypoints; %All the other waypoints


%% Generating the RRT 
%RRT parameters
RRTparam.p = 2;
RRTparam.stepsize = 0.5;
RRTparam.maxAttempts = 2000;
RRTparam.sampFunc = @(n) uniRand(n,mapBounds);

[RRT,path,goal] = buildRRT(mapEight,mapBounds,R,initWaypoint,goalPoints,RRTparam);
RRTparam.RRT = RRT;
RRTparam.path = path;
RRTparam.goal = goal;

%% Other things to initialize
%CHANGE THIS PLEASE
covStruct.Q = 0.1;
covStruct.R = 0.1;
%NOTE TWO DIFFERENT Rs HERE, don't confuse


%% Other miscellaneous useful parameterse
miscStruct.wallThickness = wallThickness;
miscStruct.angles = linspace(27,-27,9) .* (pi/180); %in rad/s
miscStruct.sensorOrigin = sensorOrigin;
miscStruct.R = R;

%% For debugging
%plotting the maps
colorsRRT = 'rycgbr';
colorsPath = 'rycgbr';

miscStruct.colorsRRT = colorsRRT;
miscStruct.colorsPath = colorsPath;

%plotMap4(mapFour,1); %Includes the opt walls
figure(1);
clf;
plotMap8(mapEight,mapBounds,1); %Includes the opt walls
hold on;
plotRRTPath(RRT,path,colorsRRT,colorsPath,1);
hold off;
end

