function wallTruth = wallCheckSequence(robotPose,depthData,miscStruct,mapStruct,covStruct)
%WALLCHECKSEQUENCE Check the optionial wall if the robot is facing the wall
%
%INPUTS: 
%   robotPose = current robot pose
%   depthData = depth measurements
%   miscStruct
%       sensorOrigin = [x_offset, y_offset], depth center rel to rob center
%       Qcovariance = covariance of the measurement function
%       angles = angles for depth sensor readings
%       wallThickness = thickness of the wall
%   mapStruct = structure that describes the map. Just need mapFour
%   covStruct = structure containing the covariances
%
%OUTPUTS:
%   wallTruth = lgical of whether or not this wall I'm checking

%Etracting information
wallThickness = miscStruct.wallThickness;
angles = miscStruct.angles;
sensorOrigin = miscStruct.sensorOrigin;
Q = covStruct.Q;

mapFour = mapStruct.mapFour; %CURRENT IDEA OF WHAT MAP LOOKS LIKE

%% Check the wall 
%Setup the annonymous function
hFun = @(robotPose) hDepthMap(robotPose,mapFour,sensorOrigin,angles,wallThickness);

wallTruth = wallOrNah(robotPose,depthData,hFun,Q);

end

