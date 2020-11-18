function [mapStruct,beaconLoc,waypointStruct] = processInput(txtFile)
%Takes a txtFile containing the map info and outputs all the relevant
%things

%INPUTS:
%mapStruct contains information about the map obstacles and boundaries
%   mapFour = map, including boundaries, in nx4 format
%   mapEight = map, excluding boundaries, in (n-4)x8 format. Adds some
%       thickness to the walls- thickness of 0.1m.
%   mapBounds = [xmin ymin xmax ymax] that define the boundaries of map
%   optWalls = mx4 containing the endpoints of the optional walls
%beaconLoc = location of beans- cx3.
%waypointStruct = contains info about the waypoints and ECwaypoints
%   waypoints- ax2, a = number of waypoints. Each row is (x,y). 
%   ECwaypoints = bx2, b = number of extra credit waypoints.


%mapFour = nx4 array that contains 
%mapWallSixteen = nx8 array that contains only the walls, not the
%   boundaries


dataStruct = importdata(txtFile);

optWalls = dataStruct.optWalls;

%nx4 containing the walls in the comp. First 4 are the outer boundaries. n = #walls
mapFour = dataStruct.map; 
mapFour = [mapFour;optWalls];

%Build the map of the wall that I like to deal with. Give it some wall
%thickness
wallThickness = 0.1; %m

[xMin, xMax, yMin, yMax] = minMaxMap(mapFour);
mapBounds = [xMin yMin xMax yMax];
mapStruct.mapEight = mapFormatChange(mapFour,mapBounds,wallThickness);

mapStruct.mapFour = mapFour;
mapStruct.mapBounds = mapBounds;
mapStruct.optWalls = optWalls;


%Beacon number, then (x,y). cx3
beaconLoc = dataStruct.beaconLoc;

waypointStruct.waypoints = dataStruct.waypoints; 
waypointStruct.ECwaypoints = dataStruct.ECwaypoints;
end