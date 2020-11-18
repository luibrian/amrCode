function VW = visitWaypointsLine(waypoints, gotopt, x, y, theta, timestep, timestepVar)
%VISITWAYPOINTSLINE: Calculate V,W to visit a series of waypoints in a line
%   INPUTS
%       Vx          x velocity in inertial frame
%       Vy          y velocity in inertial frame
%       theta       angle of the robot
%       maxSpeed    maximum speed of robot
%       timestep    estimate of how long we will travel in next step
% 
%   OUTPUTS
%       VW          Vector [forward velocity; angular velocity]

radianThreshold = 0.075;
factor = exp(-sqrt(timestepVar)*2) / timestep;

Vx = waypoints(gotopt, 1) - x;
Vy = waypoints(gotopt, 2) - y;
phi = mod(atan2(Vy, Vx) - theta, 2*pi);
if phi > pi
    phi = phi - 2*pi;
end
if abs(phi) > radianThreshold
    VW = [0, phi * 0.8 * factor];
else
    VW = [norm([Vx,Vy]) * 0.9 * factor, 0];
end
end
