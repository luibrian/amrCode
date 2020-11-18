%Brian Lui
%bl569
%Hw8

clear all;
close all;

%% Setting up parameters
%
%Processing the map and getting stuff in form I like
mapTxt = 'compMap.mat';
[mapStruct,beaconLoc,waypointStruct] = processInput(mapTxt);
mapFour = mapStruct.mapFour;
mapEight = mapStruct.mapEight;
mapBounds = mapStruct.mapBounds;
optWalls = mapStruct.optWalls;
waypoints = waypointStruct.waypoints;
ECwaypoints = waypointStruct.ECwaypoints;


%allWaypoints = waypoints;
allWaypoints = [waypoints; ECwaypoints];

initPoint = allWaypoints(1,:);
goalPoints = allWaypoints;
goalPoints(all(goalPoints ==initPoint,2),:) = [];

R = 0.13;

RRTparam.p = 2;
RRTparam.stepsize = 0.5;
RRTparam.maxAttempts = 2000;
RRTparam.sampFunc = @(n) uniRand(n,mapBounds);


%% Plotting map
%Only for debugging!
plotMap4(mapFour,1);
%plotMap4(optWalls,1);

mapEightNoOptWalls = mapEight(1:end-length(optWalls(:,1)),:);
mapEight = mapEightNoOptWalls; %CONTAINS NO OPT WALLS

plotMap8(mapEight,mapBounds,2);
bloatedMap = createBloatedMap(mapEight,R);
plotMap8(bloatedMap,mapBounds,2);
plotMap8(bloatedMap,mapBounds + [R R -R -R],3);

plot(allWaypoints(:,1), allWaypoints(:,2),'kx');
text(initPoint(1),initPoint(2),'1','FontSize',14);



%% Running things- testing
%
% RRT point
%

colorsRRT = 'ycgbr';
colorsPath = 'ycgbr';

i = 1;
j = 2;
while ~isempty(goalPoints)
    i = 1+ mod(i,length(colorsRRT));
    [RRT,path,goal] = buildRRT(mapEight,mapBounds,R,initPoint,goalPoints,RRTparam);
    path
    goalPoints(all(goalPoints==goal,2),:) = [];
    initPoint = goal;
    
    plotRRTPath(RRT,path,colorsRRT,colorsPath,i);
    text(goal(1),goal(2),num2str(j),'FontSize',14);
    i = i+1;
    j = j+1;
end 



%% Plotting the simulator things
%NOTE: THIS HAS ITS OWN PLOT- Comment out the first section here
%{

%Building the 
wallmap = importdata('cornerMap.mat');
[xMin, xMax, yMin, yMax] = minMaxMap(wallmap);
mapLimits = [xMin yMin xMax yMax];
thickness = 0.5;

%map = mapFormatChange(wallmap,mapLimits,thickness); %putting it into nx16 format
%save('cornerMapSixteen.txt', 'map','-ascii');

figHdl = 1;
plotMap4(wallmap,figHdl);
%plotMap('spiralMapSixteen.txt',mapLimits,figHdl);
%plotMap('cornerMapSixteen.txt', mapLimits, figHdl);
box on;
xlabel('x position');
ylabel('y position');

%From the data
%
dataStoreStruc = load('rrtRunGood.mat');
dataStore = dataStoreStruc.dataStore;
plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3));

RRT = dataStore.RRT;
path = dataStore.path;
nodePos = RRT.Nodes{:,2};
nodeX = nodePos(:,1);
nodeY = nodePos(:,2);
figHdl = plot(RRT,'XData',nodeX,'YData',nodeY);

%Plotting the path
highlight(figHdl,path(:,1),'EdgeColor','g','LineWidth',5);
figHdl.NodeLabel = {};
%
%}


