function bloatedMap = createBloatedMap(map,R)
%CREATEBLOATEDMAP Adds the radius of the robot into the map obstacles
% Only good for rectangular objects. 
%
%INPUTS:
%   map = nx16 list of obstacles
%   R = radius of the robot
%
%OUTPUTS:
%   bloatedMap = map with obstacles but bloated

[Vcell,~,~,~] = obsGraph(map);

bloatedMap = zeros(size(map));


%Push vertices left of center more left, right of center more right, etc.
for i = 1:length(Vcell)
    obstacleVertices = Vcell{i,1};
    obstacleXs = obstacleVertices(:,2);
    obstacleYs = obstacleVertices(:,3);
    
    xAverage = mean(obstacleXs);
    yAverage = mean(obstacleYs);
    centerPoint = [xAverage yAverage];
    
    for j = 1:length(obstacleVertices(:,1))
        obstacleX = obstacleXs(j);
        obstacleY = obstacleYs(j);
        
        xSign = sign(obstacleX - centerPoint(1));
        ySign = sign(obstacleY - centerPoint(2));
                
        newX = obstacleX + sign(xSign)*R;
        newY = obstacleY + sign(ySign)*R;
        
        bloatedMap(i,2*j-1) = newX;
        bloatedMap(i,2*j) = newY;
    end
    
end

end

