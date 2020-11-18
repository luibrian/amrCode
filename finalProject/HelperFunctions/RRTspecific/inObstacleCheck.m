function inObs = inObstacleCheck(sample,Vcell)
%INOBSTACLECHECK check if sample [x,y] in an obstacle, defined by Vcell.
%
%INPUTS:
%   sample- 1x2 [x,y] of the sample point
%   Vcell- vertices that make up the cell

xq = sample(1);
yq = sample(2);

inObs = 0;
for i = 1:length(Vcell)
    %Pulling the x and y position of the obstacle
    obstacle = Vcell{i,1};
    xv = obstacle(:,2);
    yv = obstacle(:,3);

    in = inpolygon(xq,yq,xv,yv);
    inObs = inObs || in;

end

end

