function robPoseGlobal = robotInitLocalization(rotRate,depthData,mapStruct,beaconLoc,waypointStruct,covStruct,miscStruct)
%ROBOTLOCALIZATION Summary of this function goes here
%  
%INPUTS:
%   rotRate = rotation rate of robot when generating data
%   depthData
%   beaconData - nx5 beacon daat or empty
%   mapStruct - struct containing map information
%   beaconLoc - location of all the beacons in the map
%   waypointStruct - struct containing waypoint info
%   covStruct - struct containing the covariances
%   miscStruc - other miscellaneous but useful information
%
%OUTPUTS:
%   robPoseGlobal = robot pose in the global frame

%depthData,mapStruct,beaconLoc,waypointStruct,covStruct,miscStruct

%figure out which beacons are visible at each waypoint
possibleStartPos = waypointStruct.waypoints;

mapEight = mapStruct.mapEight;
[~, ~,Vmat,Emat] = obsGraph(mapEight);
for i = 1:length(possibleStartPos(:,1))
    point1xy = possibleStartPos(i,:);
    
    intersectArray = zeros(length(beaconLoc(:,1)),1);
    for j = 1:length(beaconLoc(:,1))
        point2xy = beaconLoc(j,2:3);
        intersectObsTruth = intersectObstacleTruth(point1xy,point2xy,Vmat,Emat);
        intersectArray(j) = intersectObsTruth;
    end
    
    goodBeacon = ~any(intersectArray)
end


%{
if ~isempty(beaconData)
    %Figure out which beacons were seen
    beaconsSeen = unique(beaconData(:,3));
    
end   
%}


%If I was at a waypoint, which beacons would I see


end

