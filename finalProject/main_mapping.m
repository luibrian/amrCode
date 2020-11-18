%Brian Lui
%bl569
%

close all;
clear all;

%Processing the map and getting stuff in form I like
mapTxt = 'compMap.mat';
[mapStruct,beaconLoc,waypointStruct] = processInput(mapTxt);
mapFour = mapStruct.mapFour;
mapEight = mapStruct.mapEight;
mapBounds = mapStruct.mapBounds;
optWalls = mapStruct.optWalls;
waypoints = waypointStruct.waypoints;
ECwaypoints = waypointStruct.ECwaypoints;


%Setting up map, annonymous sensor function, loop time
angles = linspace(27,-27,9);  %in degrees
sensorOrigin = [0.1, -0.035]; %This will be [x_offset y_offset] given
wallThickness = 0.1;
%mapWithOptWalls = [mapFour;optWalls];
hFun = @(robotPose) hDepthMap(robotPose,mapFour,sensorOrigin,angles,wallThickness);

Qcovariance = 0.1;
R = 0.13;

covStruct.Q = Qcovariance;
covStruct.R = 0.1;

miscStruct.wallThickness = wallThickness;
miscStruct.angles = linspace(27,-27,9); %in degrees
miscStruct.sensorOrigin = [0 0]; %CHANGE THIS TO REFLECT THE INPUT TO FILE
miscStruct.R = R;

RRTparam.p = 2;
RRTparam.stepsize = 0.5;
RRTparam.maxAttempts = 2000;
RRTparam.sampFunc = @(n) uniRand(n,mapBounds);


%robPose = [1 -0.5 pi]
robPose = [1 0 -pi/2];
sensorLoc = [0 0];


%THIS GOES TOWARDS THE INITIALIZATION OF THE FIRST RRT
%Add 2 points on either side of the wall as a possible goalPoint from RRT
%NOTE: TOSS OUT THE OPT WALLS WHEN DOING THIS PARTICULAR ONE
mapEightNoOptWalls = mapEight(1:end-length(optWalls(:,1)),:);
mapBloatedNoOptWalls = createBloatedMap(mapEightNoOptWalls,R);
[Vcell, Ecell,Vmat,Emat] = obsGraph(mapBloatedNoOptWalls);

wallThickness = 0.1;
R = 0.13;
buffer =0.2;
[waypointsOptWall4,toCheckWaypoints] = makeOptWallWaypoints(optWalls,mapFour,mapBounds,R,wallThickness,buffer,Vmat,Emat)

totalWaypoints = [waypoints; ECwaypoints; toCheckWaypoints];

waypointStruct.optWaypointsArray4 = waypointsOptWall4;   %Waypoints for opt walls
waypointStruct.totalWaypoints = totalWaypoints;         %All waypoints to check
waypointStruct.visitedWaypoints = [];

waypointStruct.waypointChecked = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure(10)
plot(toCheckWaypoints(:,1),toCheckWaypoints(:,2),'x');
hold on;
%
plotMap4(mapFour,10);
plotMap8(mapEight,mapBounds,10);
bloatedMap = createBloatedMap(mapEight,0.13);
plotMap8(bloatedMap,mapBounds,10);


waypointToCheck = toCheckWaypoints(end,:);
[optWallWaypointTruth,wallToCheck,waypointPair] = optWallWaypointCheck(waypointToCheck,waypointsOptWall4,optWalls);

%Check the turning



%Testing if he robot can detect a wall from a stat pos
%
%Wall is there data
%txt = 'optWallThere1_straightFace_stat.mat';
%txt = 'optWallThere1_angleFace_vel.mat';
%txt = 'optWallThere1_angleFace_stat.mat';
%txt = 'optWallThere1_straightFace_stat2.mat';
%txt = 'optWallThere1_straightFace_vel.mat';

%Wall is not there data
%txt = 'optWallNotThere1_straightFace_stat.mat';
txt = 'optWallNotThere1_straightFace_stat2.mat';
%txt = 'optWallNotThere2_straightFace_stat.mat';
%txt = 'optWallNotThere2_straightFace_vel.mat';
%txt = 'optWallNotThere1_angleFace_stat.mat';
%txt = 'optWallNotThere1_angleFace_vel.mat';


dataStore= importdata(txt);
%

%TESTING updateMapWaypoints
%
%Wall is not there data
depthData = dataStore.rsdepth(:,3:11);
robPose = mean(dataStore.truthPose(:,2:4));

wallTruth = wallOrNah(robPose,depthData, hFun, Qcovariance);
wallTruth
wallChecked = wallToCheck;
%{
%[mapStruct,waypoints] = updateMapWaypoints(waypointChecked,mapStruct,waypoints,waypointsOptWall4,wallTruth)
waypointStruct
mapStruct
[mapStruct,waypointStruct] = updateMapWaypoints(waypointToCheck,optWallWaypointTruth,waypointPair,wallChecked,wallTruth,mapStruct,waypointStruct)
%}