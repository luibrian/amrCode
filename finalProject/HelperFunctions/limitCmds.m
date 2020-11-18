function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Miller, Jeremy

leftVel = fwdVel + angVel * wheel2Center;
rightVel = fwdVel - angVel * wheel2Center;

maxWheelVel = max(abs(leftVel), abs(rightVel));
    
if maxWheelVel > maxV
    
    leftVel = leftVel * maxV/maxWheelVel;
    rightVel = rightVel * maxV/maxWheelVel;
   
    solution = [1 wheel2Center; 1 -wheel2Center] \ [leftVel; rightVel];
    cmdV = solution(1);
    cmdW = solution(2);
else
    cmdV = fwdVel;
    cmdW = angVel;
end
 





