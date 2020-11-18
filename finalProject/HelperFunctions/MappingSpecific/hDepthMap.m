function depthMeasurement = hDepthMap(robotPose,map,sensorOrigin,angles,wallThickness)
%hDepth Predict the measurement given the depth predict measurements

ranges = rangePredict(robotPose,map,sensorOrigin,angles);
depthMeasurement = depthPredict(ranges);

depthMeasurement = depthMeasurement- wallThickness/2;
end

