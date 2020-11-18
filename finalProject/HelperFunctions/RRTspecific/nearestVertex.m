function [nearVertex, weight] = nearestVertex(sampleXY,p,graph)
%NEARESTVERTEX Finds the nearest vertex to samplePoint
%
%INPUTS:
%   sampleXY = [x,y] point of the sample
%   graph = contains nodes and vertices that make up the RRT
%   p = which el_norm to compute
%   Vmat = list of all the vertices that make up the obstacles
%   Emat = list of all the edges that make up the obstacles
%
%OUPUTS:
%   nearVertex = nearest vertex on the graph to the samplePoint. If there
%       are none, return [].
%   weight = distance between the nearVertex and samplePoint. If there are
%       none, return 0.

nodeNum = graph.Nodes{:,1};
nodeXYall = graph.Nodes{:,2};

%First col is index num, second col is weight
distances = zeros(length(nodeNum),2);
distances(:,1) = linspace(1,length(nodeNum),length(nodeNum));
for i = 1:length(nodeNum)
    nodeXY = nodeXYall(i,:);
    
    dist = elpnorm(sampleXY,nodeXY,p);
    distances(i,2) = dist;
end

%Finding which one is the closest good point
distancesSorted = sortrows(distances,2);

nearVertex = distancesSorted(1,1);
weight = distancesSorted(1,2);


end
