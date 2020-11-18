function [RRT,path,goal] = buildRRTpoint(map,mapBounds,initPoint,goalPoints,RRTparam)
%BUILDRRTpoint Generates waypoints using a rapidly-explored random tree
%that connect startLoc and goalLoc
%
%INPUTS:
%   map = mx8 vector of the boundaries
%   mapBounds = [xMin yMin xMax yMax] that describe map boundary
%   initPoint = [x,y] of the starting location of robot
%   goalPoints = nx2 array [x,y] of many possible desired finish position of the robot
%   RRTparam- contains
%       stepsize = distance to travel every time
%       p = which el_p norm to compute
%       maxAttempts = how many attempts before giving up sampling
%       sampFunc = sampling function.
%
%OUTPUTS:
%   RRT. Rapidly-exploring random tree. It's a graph containing vertices a
%       edges. 
%   path = nx3. [vertex num, x,y] connect initPoint to goalPoint

[Vcell,Ecell,Vmat,Emat] = obsGraph(map);

%So it doesn't intersect the boundary
Vboundary = [1 mapBounds(1) mapBounds(2);...
    2 mapBounds(3) mapBounds(2);...
    3 mapBounds(3) mapBounds(4);...
    4 mapBounds(1) mapBounds(4)];
Eboundary = [1 2; 2 3; 3 4; 4 1];

stepsize = RRTparam.stepsize;
maxAttempts = RRTparam.maxAttempts;
p = RRTparam.p;
sampFunc = RRTparam.sampFunc;

%Initializing the graph
RRT = graph();
nodeTable = table(1,initPoint,'VariableNames',{'Var1', 'Var2'});
RRT = addnode(RRT,nodeTable);


m = 1;
sampleNum = 2; %because initial node is 1, goal point is 2
notDone = 1;
while (m < maxAttempts) && notDone
    sampleXY = sampFunc(1);
    
    inObs = inObstacleCheck(sampleXY,Vcell);
    
    %If no intersection, try adding to path
    if ~inObs
        [nearVertex, weight] = nearestVertex(sampleXY,p,RRT);
        nearVertexXY = RRT.Nodes{nearVertex,2}; 
       
        %Move by stepSize from nearVertex to sampleXY
        dirToTravel = sampleXY - nearVertexXY;
        dirUnit = dirToTravel/norm(dirToTravel);
        trySample = nearVertexXY + dirUnit*stepsize;
        
        %check if this point intersects any obstacles
        intersectObsTruth = intersectObstacleTruth(nearVertexXY,trySample,Vmat,Emat);
        %check if it intersects map boundary
        intersectBoundTruth = intersectObstacleTruth(nearVertexXY,trySample,Vboundary,Eboundary);
        
        intersectTruth = intersectObsTruth || intersectBoundTruth;
        
        if ~intersectTruth
            %Add it to the graph
            newEdge = [nearVertex sampleNum];
            nodeTable = table(sampleNum,trySample,'VariableNames',{'Var1', 'Var2'});
            edgeTable = table(newEdge,stepsize,'VariableNames',{'EndNodes' 'Weight'});
            RRT = addnode(RRT,nodeTable);
            RRT = addedge(RRT,edgeTable);
            sampleNum = sampleNum + 1;
            
            %RRT.Nodes
            
            %Try to see if the new vertex connects to the goalPoint
            notDoneArray = ones(length(goalPoints(:,1)),1);
            for i = 1:length(goalPoints(:,1))
                goalPoint = goalPoints(i,:);
                notDoneArray(i) = intersectObstacleTruth(trySample,goalPoint,Vmat,Emat);
                
                %if ~notDoneArray(i)
                %    goalPointReached = goalPoint;
                %end
            end
            notDone = all(notDoneArray);
            
            if ~notDone
                %notDoneArray
                %pointIndex = ~logical(notDoneArray)
                goal = goalPoints(~notDoneArray,:);
                
                goal = [goal(1) goal(2)];
                
                goalNum = sampleNum;
               
                nodeTable = table(goalNum,goal,'VariableNames',{'Var1', 'Var2'});
                RRT = addnode(RRT,nodeTable);
                
                newEdge = [sampleNum-1 goalNum];
                edgeTable = table(newEdge,stepsize,'VariableNames',{'EndNodes' 'Weight'});
                RRT = addedge(RRT, edgeTable);         
                
            end
            
        end
    end
    m = m+1;
end

if notDone
    path = [];
    goal = [];
else
    %% Finding shortest path from initial to goal
    pathNum = shortestpath(RRT,1,sampleNum);
    %Returning output in desired format
    path = zeros(length(pathNum),3);
    for i = 1:length(pathNum)
        nodePos = RRT.Nodes{pathNum(i),2};
        path(i,:) = [pathNum(i) nodePos];
    end
end
end

