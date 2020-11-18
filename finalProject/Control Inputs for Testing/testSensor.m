function[dataStore] = testSensor(CreatePort,DistPort,TagPort,tagNum,maxTime)
%RRTPLANNER Uses visitWaypoints to visit waypoints from RRT
%
%   dataStore = MOTIONCONTROL(createPort,DistPort,TagPort,tagNum,maxTime) runs
%   
%   INPUTStype
%       CreatePort  Create port object (get from running RoombaInit)
%       DistPort    Depth ports object (get from running CreatePiInit)
%       TagPort     Tag ports object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%   
%   OUTPUTS
%       dataStore   struct containing logged data
%
%   NOTE: Assume differential-drive robot whose wheels turn at a constant
%   rate between sensor readings

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DistPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 5
    maxTime = 500;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'RRT', [],...
                   'path',[]);

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%{
%Parameters for RRT
goalPoint = [-1 -1];
R = 0.2;    %robot radius
thickness = 0.1; %thickness to make the walls

file4 = importdata('lab5mapOptWall.mat');
%file4 = importdata('lab5map.mat');
[xMin, xMax, yMin, yMax] = minMaxMap(file4);
mapBounds = [xMin yMin xMax yMax];
%map = importdata('lab5map16.txt');
map = importdata('lab5mapOptWall_16.txt');

RRTparam.stepsize = 0.5;
RRTparam.p = 2;
RRTparam.maxAttempts = 1000;
RRTparam.sampFunc = @(n) uniRand(n,mapBounds);

%parameters for differential drive
epsilon = 0.5;
closeEnough = 0.2;

waypoints = [];
%}

tic
while toc < maxTime

    %% READ & STORE SENSOR DATA
    [noRobotCount, dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
%{
    truthPose = dataStore.truthPose(end,2:4);
    initPoint = truthPose(1:2);
    
    %initializing the robot
    if isempty(waypoints)
        [RRT, path] = buildRRT(map,mapBounds,R,initPoint,goalPoint,RRTparam);
        dataStore.RRT = RRT;
        dataStore.path = path;
        waypoints = path(:,2:3);
        gotopt = 1;
        
    end
    
    % CONTROL FUNCTION (send robot commands)
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
       SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        if gotopt <= length(waypoints(:,1))
            [cmdV,cmdW] = visitWaypoints(waypoints,truthPose,epsilon,gotopt,closeEnough);
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
            if (cmdV == 0) && (cmdW == 0)
                gotopt = gotopt + 1;
                SetFwdVelAngVelCreate(CreatePort, 0, 0 );
                
                save('rrtRunGood.mat','dataStore')
            end
        else
            SetFwdVelAngVelCreate(CreatePort,0,0 );

        end
    end
%}
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

