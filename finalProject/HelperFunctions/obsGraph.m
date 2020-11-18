function [Vcell, Ecell,Vmat,Emat] = obsGraph(map)
%OBSGRAPH Given a map (nx8), returns a cell array V and E containing the
%vertices and edges of each obstacle. 

%INPUTS:
%   map: nx8 containing the vertices of the obstacles in the map
%
%OUTPUTS:
%   Vcell = cell array, each row n corresponds to obstacle n.
%       Each row contains a vector of the location of the vertices
%       and the vertex number in obstacle n, ie. [1 5 5] = vertex 1 located
%       at (5,5).
%   Ecell = cell array, each row n corresponds to the edges that make up
%       obstacle n.
%   Vmat = matrix containing all the V
%   Emat = matrix containing all the E

numObs = length(map(:,1)); 
maxPoints = length(map(1,:)) /2;

%% Filling in Vcell
Vcell = {}; %Each obs, defined by index num, forms a matrix, which occupy a row in Vcell
nodeNum = 1;
for i = 1:numObs   %going through each obstacle
    verticesInOb = []; 
    for j = 1:maxPoints %going through each pt
        if (map(i,2*j-1) ~= 0 || map(i,2*j) ~=0) 
            vertex = [map(i,2*j-1) map(i,2*j)];
            verticesInOb = [verticesInOb; nodeNum vertex]; 

            nodeNum = nodeNum + 1;
        end
    end
    Vcell{i,1} = verticesInOb;
end

%% Filling in Ecell for the obstacle boundaries
Ecell = cell(size(Vcell)); %contains the edges, which vertices connect to each other.
for i = 1:length(Vcell)
    V = Vcell{i};
    E = zeros(length(V),2);
    
    %n-1 sides of the obstacle
    for j = 1:length(V)-1
        edgeToAdd = [V(j,1), V(j+1,1)];
        E(j,:) = edgeToAdd;
    end
    
    %Adding the last side of the obstacle
    edgeToAdd = [V(j+1,1), V(1,1)];
    E(j+1,:) = edgeToAdd;
    
    Ecell{i,1} = E;
end

%% Extracting the map info
Vmat = [];  %matrix that contains all the vertices
Emat = [];  %matrix that contains all the edges
for i = 1:length(Vcell)
    Vmat = [Vmat; Vcell{i}]; 
    Emat = [Emat; Ecell{i}];
end

end

