function [inSight,angleToTurn] = lineOfSight(robPose,sensorLoc,map,wallToCheck)
%LINEOFSIGHT Checks if the opt wall is visible from the current location
%SHOULD CALL THIS ANGLE TO ROTATE 
%
%INPUTS:
%   robPose = current robot pose. 1x3 or 3x1
%   sensorLoc = location of depth sensor relative to the center of robot.
%       1x2 or 2x1. [offset_x offset_y]
%   map = mx8 array of all the obstacles in the map. 
%   wallToCheck = wall to check if it's there or not. 1x4
%
%OUTPUTS:
%   inSight = logical whether the desired wall is visible from the robot's
%       current location
%   angleToTurn = angle the robot needs to turn to be able to see the wall
%       positive angle = CCW turn, negative angle = CW turn

inSight = 0;
angleToTurn = 0;

%% Finding the position of the sensor in the global frame
sensorLoc = reshape(sensorLoc, [length(sensorLoc), 1]);

theta = robPose(3);
R = [cos(theta) -sin(theta); sin(theta), cos(theta)];
sensorGlobal = R*sensorLoc;
x1 = robPose(1) + sensorGlobal(1);
y1 = robPose(2) + sensorGlobal(2);

%% Break the optional wall down into discrete points
npoints = 5;

p1 = wallToCheck(1:2);
p2 = wallToCheck(3:4);
wallBreaks = linspace(0,1,npoints)';
optWallSegments = p1 + wallBreaks .*(p2-p1);


%% Check if the robot + any of the optWall segments intersect any map obstacles
%go through every point in optWallSegment
optWallSegmentsSee = zeros(length(optWallSegments(:,1)),1);
for i = 1:length(optWallSegments(:,1)) 
    x2 = optWallSegments(i,1);
    y2 = optWallSegments(i,2);

    j = 1;
    see = 1;
    %go through obstacles j 
    while j <= length(map(:,1)) && see
        k = 1;
        %check all the walls in obstacle j
        while k <= length(map(1,:))/2 && see
            if k == length(map(1,:))/2
                g = 1;
            else
                g = k+1;
            end
            
            x3 = map(j,2*k-1);
            y3 = map(j,2*k);
            x4 = map(j,2*g-1);
            y4 = map(j,2*g);
            [isect,x,y,~]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
            
            %
            if isect
                %if it doesn't intersect at the endpoint of the obstacle
                if ((x == x3) && (y == y3)) || ((x == x4) && (y == y4))
                    see = true;
                else
                    see = false;
                end
            end
            
            k = k + 1;
        end
        j = j+1;
    end


    optWallSegmentsSee(i) = see;
end

%% Doing what it needs to do if point is visible to robot
if any(optWallSegmentsSee)
    %FIND THE MIDDLE ANGLE
    inSight = any(optWallSegmentsSee);
    
    visibleOptWall = optWallSegments(logical(optWallSegmentsSee),:);
    wallMiddle = mean(visibleOptWall);
    
    robotAngle = robPose(3);
    
    distToWall = wallMiddle - robPose(1:2);
    angleToOptWall = atan2(distToWall(2), distToWall(1));
    if angleToOptWall < 0
        angleToOptWall = angleToOptWall + 2*pi;
    end
    
    angleToTurn = angleToOptWall - robotAngle;
    %Turn always in the direction that requires the smallest turn
    if abs(angleToTurn) > pi 
        angleToTurn = -(2*pi - angleToTurn); 
    end
end

plot(robPose(1),robPose(2),'x');
plot(wallMiddle(1), wallMiddle(2),'x');
wallMiddle

end

