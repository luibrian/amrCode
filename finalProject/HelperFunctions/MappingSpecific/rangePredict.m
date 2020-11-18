function[range] = rangePredict(robotPose,map,sensorOrigin,angles)
% RANGEPREDICT: predict the range measurements for a robot operating
% in a known map.
% 
%   RANGE = RANGEPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES) returns 
%   the expected range measurements for a robot operating in a known 
%   map. These range measurement corresponds to the distance between the
%   sensor and the closest obstacle. Assume the sensor reference frame is 
%   not rotated with respect to the robot-fixed reference frame
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame. Given in rads
%
%   OUTPUTS
%       range       	K-by-1 vector of ranges (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Lui, Brian

%angles = angles*pi/180;
%if sensorOrigin is inputted as 2x1 instead of 1x2
sensorOrigin = reshape(sensorOrigin, [1,2]);

range = inf(length(angles),1);      %preallocaing the range matrix
rangeMaxLength = 30;     %max range of the range sensor in distance.

%Given pose of the robot. Given in inertial frame
robotX = robotPose(1);
robotY = robotPose(2);
robotTheta = robotPose(3);

%Rotating the robot into the inertial frame
rot_rob2inert = [cos(robotTheta), -sin(robotTheta);...
    sin(robotTheta), cos(robotTheta)];

%Converting location of range sensor into inertial frame
sensDist_inertial = rot_rob2inert*sensorOrigin';

sensor_robX = robotX + sensDist_inertial(1);     %loc of robot + dist to sensor
sensor_robY = robotY + sensDist_inertial(2);

x3 = sensor_robX;
y3 = sensor_robY;

%Converting the robot sensor points into global
%maxRangeXArray = zeros(length(angles),1);
%maxRangeYArray = zeros(length(angles),1);

for k = 1:length(angles)
    %Converting the distance of the sensor max range into inertial
    sensorFarDistX_rob = rangeMaxLength*cos(angles(k));
    sensorFarDistY_rob = rangeMaxLength*sin(angles(k));
    sensorFarDist_inertial = rot_rob2inert*[sensorFarDistX_rob, sensorFarDistY_rob]';
    
    %The far distance of the sensor max range in inertial coords
    sensorFar_inertial = sensorFarDist_inertial + [sensor_robX; sensor_robY];
    
    %Filling up the 
    x4 = sensorFar_inertial(1);
    y4 = sensorFar_inertial(2);
    
    for n = 1:length(map(:,1))       %looping through the map lines
        x1 = map(n,1); y1 = map(n,2);   %map point 1
        x2 = map(n,3); y2 = map(n,4);   %map point 2
        %x1,y1,x2,y2 from line segment 1; x3,y3,x4,y4 from line segment 2
        [isect,x,y,~]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
        
        if isect %if the two lines intersect
            dist = sqrt((x-x3)^2 + (y-y3)^2);   %x3,y3 the loc of sensor
            if dist < range(k)
                range(k) = dist;    %will always be positive number
            end
        end
    end
end



