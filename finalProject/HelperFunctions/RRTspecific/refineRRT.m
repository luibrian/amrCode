function [RRTcleaned, pathCleaned] = refineRRT(RRT,path,map)
%REFINERRT Takes the RRT output from buildRRTpoint and cleans up the path,
%more direct path taken from start to goal
%
%INPUT:
%   RRT- Rapidly-exploring random tree. It's a graph containing vertices a
%       edges. 
%   path- list of nodes to take robot from start to goal
%   map- nx16 containing the obstacles
%
%OUTPUTS:
%   RRTcleaned - RRT with some more edges
%   pathCleaned- cleaned up list of nodes to take robot from start to goal

[~,~,Vmat,Emat] = obsGraph(map);

RRTcleaned = RRT;

for i = 1:length(path(:,1)) %going through each node in the path
    vertex1 = path(i,2:3);
    for j = 1:length(path(:,1))
        if i ~= j
            vertex2 = path(j,2:3);
            intersectObsTruth = intersectObstacleTruth(vertex1,vertex2,Vmat,Emat);
            
            if ~intersectObsTruth
                newEdge = [path(i,1) path(j,1)];
                inGraph = any(findedge(RRTcleaned,newEdge(1),newEdge(2)));
                
                if ~inGraph

                    stepsize = elpnorm(vertex1,vertex2,2);
                    edgeTable = table(newEdge,stepsize,'VariableNames',{'EndNodes' 'Weight'});

                    RRTcleaned = addedge(RRTcleaned, edgeTable);  
                end
            end
        end
    end
    
end

%if the two RRTs are the same or not
if isisomorphic(RRT,RRTcleaned)
    pathCleaned = path;
else
    pathNum = shortestpath(RRTcleaned,path(1,1),path(end,1)); %,'Method','unweighted')
    pathCleaned = zeros(length(pathNum),3);
    for i = 1:length(pathNum)
        nodePos = RRT.Nodes{pathNum(i),2};
        pathCleaned(i,:) = [pathNum(i) nodePos];
    end
end


%If the number of nodes is small, I can compute all the edges that connect
%all the edges, then refind the best path from start to finish

end

