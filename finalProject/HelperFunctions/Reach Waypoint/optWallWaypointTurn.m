function [angleGlobal,robPoseTurned] = optWallWaypointTurn(robPose,wallToCheck)
%OPTWALLWAYPOINTTURN If on an optional wall waypoint, turn towards the wall
%
%INPUTS:
%   robPose = current robot pose. 1x3 or 3x1. Should be one of the many
%       optWall waypoints, or very close to it.
%   wallToCheck = wall to check if it's there or not. 1x4
%
%OUTPUTS:
%   angleToTurn = angle the robot needs to turn to be able to see the wall
%       positive angle = CCW turn, negative angle = CW turn
%   angleGlobal =angle of the robot desired now in the global frame
%   robPoseTurned = robotPose after the turn


%angleToTurn = 0;
robotAngle = robPose(3);

wallMidpoint = [mean([wallToCheck(1),wallToCheck(3)]), mean([wallToCheck(2),wallToCheck(4)])];

%Finding the angle to turn
distToWall = wallMidpoint - robPose(1:2);
angleToOptWall = atan2(distToWall(2), distToWall(1));
if angleToOptWall < 0
    angleToOptWall = angleToOptWall + 2*pi;
end

angleToTurn = angleToOptWall - robotAngle;
%Turn always in the direction that requires the smallest turn
if abs(angleToTurn) > pi 
    angleToTurn = -(2*pi - angleToTurn); 
end

%Updating the robot position
angleGlobal = robPose(3) + angleToTurn;

robPoseTurned = robPose;
robPoseTurned(3) = robPoseTurned(3) + angleToTurn;

end

