function[dS] = finalCompetition(CreatePort,DistPort,TagPort,tagNum,maxTime,offset_x,offset_y)
%The master run file to do all the magick

def_offset_x = 0.13;

if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DistPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
    offset_x = def_offset_x;
    offset_y = 0;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
    offset_x = def_offset_x;
    offset_y = 0;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 500;
    offset_x = def_offset_x;
    offset_y = 0;
elseif nargin < 5
    maxTime = 500;
    offset_x = def_offset_x;
    offset_y = 0;
end

global dataStore;
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'mu', [], ...
                   'sigma', [], ...
                   'beacon', []);
               
               
% Initial Localization
[px, py, pt] = OverheadLocalizationCreate(tagNum);
mu = [px; py; pt];
initWaypoint =[-2.4300 -0.5000];

% Map Constants
mapTxt = 'compMap.mat';
sensorOrigin = [offset_x, offset_y];
[mapStruct,beaconLoc,waypointStruct,RRTparam,covStruct,miscStruct] = initialization(mapTxt,initWaypoint,sensorOrigin)

%{
input = importdata('compMap.mat');
map = input.map;
waypoints = input.waypoints;
ECwaypoints = input.ECwaypoints;
beaconLoc = input.beaconLoc;

% Brian Constants
mapStruct = processInput(input);
mapFour = mapStruct.mapFour;
mapEight = mapStruct.mapEight;
mapBounds = mapStruct.mapBounds;
optWalls = mapStruct.optWalls;
%}

% More Map Constants
%
numWaypoints = size(waypointStruct.waypoints,1);
numBeacons = max(beaconLoc(:,1));
map = mapStruct.mapFour;

%numWaypoints = size(waypoints, 1);
%numBeacons = max(beaconLoc(:,1));
map = [map; 0.00, 0.00, 0.000, -0.700];
xs = [map(:, 1); map(:, 3)];
ys = [map(:, 2); map(:, 4)];
mapLimits = [min(xs), min(ys), max(xs), max(ys)];

vxMin = mapLimits(1);
vyMin = mapLimits(2);
vxMax = mapLimits(3);
vyMax = mapLimits(4);
bnd = max((vxMax-vxMin) / 20, (vyMax-vyMin) / 20);
axis([vxMin-bnd vxMax+bnd vyMin-bnd vyMax+bnd])
axis equal
%

% Localization Constants
%sensorOrigin = [offset_x, offset_y];
%angles = linspace(27*pi/180, -27*pi/180, 9)';
minDepth = 0.175;
maxDepth = 10;
angles = miscStruct.angles;


% RRT Initialization
RRT = RRTparam.RRT;
path = RRTparam.path;
goal = RRTparam.goal;
rrtPoints = path(:,2:3);

%f = @() uniformRandomSample(mapLimits);
%goal = [2.43,0.92];
%radius = 0.2;
%stepsize = 0.5;

%n x (x,y)
%rrtPoints = buildRRTA(map, mapLimits, [px,py], goal, radius, f, stepsize);
wI = 1;
feedbackLinEps = 0.15;
closeEnough = 0.05;

% Control Constants
maxSpeed = 0.2;
h = NaN;
time = 0;
timestep = 0.5; %0.1 % Prior estimate of time between iterations
timestepVar = 0.001;  % Prior estimate of the variance of time between iterations
noRobot = 0;
tic;
lastCircle = toc;

justReached = 1;
state = 1;

while toc < maxTime

    % Control Function
    timestep = 0.8*timestep + 0.2*(toc-time);
    timestepVar = 0.8*timestepVar + 0.2*((toc-time)-timestep)^2;
    
    if noRobot >= 3 || toc < 1
        VW = [0, 0];
    elseif all(dataStore.rsdepth(end, 3:11) == 0)
        VW = [-1, 0.1];
    elseif state ==1
        x = mu(1);
        y = mu(2);
        theta = mu(3);
        way_x = rrtPoints(wI, 1);
        way_y = rrtPoints(wI, 2);
        if (x - way_x)^2 + (y - way_y)^2 < closeEnough^2
            wI = wI + 1;
        end
        if wI > size(rrtPoints, 1)
            %Reached the goal- check the waypoints and stuff
            optWaypointsArray4 = waypointStruct.optWaypointsArray4;
            optWalls = mapStruct.optWalls;

            %Initializing some variables
            wallTruth = 1;
            waypointToCheck = RRTparam.goal;
            
            %Check whether waypoint is an opt wall waypoint
            [optWallWaypointTruth,wallToCheck,waypointPair] = optWallWaypointCheck(waypointToCheck,optWaypointsArray4,optWalls);

            if optWallWaypointTruth
                [angleGlobal,robPoseTurned] = optWallWaypointTurn(robPose,wallToCheck);
                newPose = robPoseTurned;
                VW = [0, angleToTurn * 0.8 * exp(-sqrt(timestepVar)*2) / timestep];
                state = 2;
            else 
                state = 3;
                VW = [0 0];
            end
        else
            % VW = visitWaypoints(waypoints, wI, x, y, theta, feedbackLinEps);
            VW = visitWaypointsLine(rrtPoints, wI, x, y, theta, timestep, timestepVar);
        end
    elseif state == 2 %Turn to face
        if abs(mu(3) - angleGlobal) < 0.15 %0.075
            state = 3;
            VW = [0 0];
            startDepthTime = length(dataStore.rsdepth(:,1));
            timeToWait = toc + 3
        end
    elseif state == 3 %Update everything and the new RRT
        if optWallWaypointTruth && toc < timeToWait %Stop and stare for 3 secs
        else
            if optWallWaypointTruth
                depthData = dataStore.rsdepth(startDepthTime+1:end,3:end);
                wallTruth =  wallCheckSequence(robotPose,depthData,miscStruct,mapStruct,covStruct);
            else %regular waypoint
                actualWaypointsSequence(CreatePort);
            end

            %Update the map and waypoints
            wallChecked = wallToCheck;
            [mapStruct,waypointStruct] = updateMapWaypoints(waypointToCheck,optWallWaypointTruth,waypointPair,wallChecked,wallTruth,mapStruct,waypointStruct,miscStruct);

            %Generate a new RRT
            mapEight = mapStruct.mapEight;
            mapBounds = mapStruct.mapBounds;
            R= miscStruct.R;
            initPoint = mu(1:2);
            goalPoints = waypointStruct.totalWaypoints;


            [RRT,path,goal] = buildRRT(mapEight,mapBounds,R,initPoint,goalPoints,RRTparam);
            RRTparam.RRT = RRT;
            RRTparam.path = path;
            RRTparam.goal = goal;

            rrtPoints = path(:,2:3);
        end
    end
    
    if ~any(isnan(VW))
        [cmdV, cmdW] = limitCmds(VW(1), VW(2), maxSpeed, 0.13);
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
    end
    time = toc;

    % Read and store sensor data
    pause(0.5);
    dataStore=rssd(CreatePort,DistPort,TagPort,tagNum,numBeacons,dataStore);
    
    % Localization
    mu = dataStore.truthPose(end, 2:4);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort,0,0);
dS = dataStore;
