%Part a
%Calculating the time it takes to update

%{
%Stationary
bumpOnly  = updateTime('sensorBump.mat','bump');
depthOnly = updateTime('sensorDepth.mat','rsdepth');
beaconOnly = updateTime('sensorOneBeacon.mat','beacon');
truthPoseOnly = updateTime('sensorTruthPose.mat','truthPose');
odometryOnly = updateTime('sensorOdometry.mat','odometry');

%Depth and beacon data, stationary
depthBeacon_depth = updateTime('sensorDepthBeacon.mat','rsdepth');
depthBeacon_beacon = updateTime('sensorDepthBeacon.mat','beacon');

%Depth and beacon data, stationary
depthBeacon_depth2 = updateTime('sensorDepth_beacon.mat','rsdepth');
depthBeacon_beacon2 = updateTime('sensorDepth_beacon.mat','beacon');

%Truthpose, odometry, depth, beacon, stationary
tp_o_d_b_truthPose = updateTime('sensorTruthPoseOdometryDepthBeacon.mat', 'truthPose');
tp_o_d_b_odometry = updateTime('sensorTruthPoseOdometryDepthBeacon.mat', 'odometry');
tp_o_d_b_depth = updateTime('sensorTruthPoseOdometryDepthBeacon.mat', 'rsdepth');
tp_o_d_b_beacon = updateTime('sensorTruthPoseOdometryDepthBeacon.mat', 'beacon');

%Moving, truthpose odometery, depth, beacon
arcMoveTP = updateTime('arcMove_tp_o_rsd_b.mat','truthPose');
arcMoveOdom = updateTime('arcMove_tp_o_rsd_b.mat','odometry');

%Moving, truthpose odometery, depth, beacon
straightMoveTP = updateTime('straightMove_tp_o_rsd_b.mat','truthPose');
straightMoveOdom = updateTime('straightMove_tp_o_rsd_b.mat','odometry');

only = [bumpOnly depthOnly beaconOnly truthPoseOnly odometryOnly]
depthBeacon = [depthBeacon_depth depthBeacon_beacon]
depthBeacon2 = [depthBeacon_depth2 depthBeacon_beacon2]
many = [tp_o_d_b_truthPose tp_o_d_b_odometry, tp_o_d_b_depth, tp_o_d_b_beacon]
arcMove = [arcMoveTP arcMoveOdom]
straightMove = [straightMoveTP straightMoveOdom]
%}

[depthOnly,covDepth] = updateTime('sensorDepth.mat','rsdepth');
[beaconOnly,covBeacon] = updateTime('sensorOneBeacon.mat','beacon');
[depthBeacon_depth,covBoth] = updateTime('sensorDepthBeacon.mat','rsdepth');

covDepth
covBeacon
covBoth

%[depthOnly_delay,covDepth] = cameraDelayTime('sensorDepth.mat','rsdepth');
%[beaconOnly_delay,covBeacon] = cameraDelayTime('sensorOneBeacon.mat','beacon');

%delayTime = [depthOnly_delay beaconOnly_delay]
%delayCov = [covDepth, covBeacon]

%Depth and beacon data, stationary
%[depthBeaconDelay_depth2,cov1] = cameraDelayTime('sensorDepth_beacon.mat','rsdepth')
%[depthBeaconDelay_beacon2, cov2] = cameraDelayTime('sensorDepth_beacon.mat','beacon')



function [timeToUpdate,cov] = updateTime(file, field)
    %reads in the file, and extracts out the average time it takes to update
    %if i = 1, average time it takes, if i = 2, average delay for camera
    load(file);
    fieldStuff = eval(['dataStore.' field]);   %stuff in the field
    timestamps = fieldStuff(:,1);
    
    %Calculating an array of timeDifs
    timeToSubtract = [0; timestamps(1:end-1)];
    timeDifs = timestamps - timeToSubtract;
    timeToUpdate = mean(timeDifs(2:end));   %first spot is useless
    
    cov = sum((timeDifs - timeToUpdate).^2) / length(timeDifs);
    
end


function [meanDelay,cov] = cameraDelayTime(file, field)
    %reads in the file, and extracts out the average time it takes to update
    %if i = 1, average time it takes, if i = 2, average delay for camera
    load(file);
    fieldStuff = eval(['dataStore.' field]);   %stuff in the field
    delays = fieldStuff(:,2);
    
    %Calculating an array of timeDifs
    meanDelay = mean(delays);
    
    cov = sum((delays - meanDelay).^2) / length(delays);
end
    
    