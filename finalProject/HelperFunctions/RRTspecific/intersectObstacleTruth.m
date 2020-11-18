function intersectObsTruth = intersectObstacleTruth(point1xy,point2xy,Vmat,Emat)
%INTERSECTOBSTACLETRUTH Does it intersect an obstacle in the map. Uses
%intersectPoint.
%
%INPUTS:
%   point1 = [x,y]. First endpoint of a line segment.
%   point2 = [x,y]. Second endpoint of the line segment.
%   Vmat - vertices of the obstacles in the map.
%   Emat - edges connecting the vertices of the obstacles in the map. 
%   
%OUTPUTS:
%   intersectObsTruth = logical of whether it intersected any obstacle
%   defined by Vmat,Emat

intersectObsTruth = 0;

x1 = point1xy(1); y1 = point1xy(2);
x2 = point2xy(1); y2 = point2xy(2);

%Now check if it intersects any objects
for k = 1:length(Emat(:,1)) 
    obsVertexNum1 = Emat(k,1);
    obsVertexNum2 = Emat(k,2);

    vertex1Pos = Vmat(obsVertexNum1,2:3);
    vertex2Pos = Vmat(obsVertexNum2,2:3);

    x3 = vertex1Pos(1); y3 = vertex1Pos(2);
    x4 = vertex2Pos(1); y4 = vertex2Pos(2);

    %If both endpoints are the same, no intersection!
    [isect,x,y,~] = intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);

    if isect 
        %If they don't intersect at the same vertex
        intersect1 = (x==x1) && (y==y1);
        intersect2 = (x==x2) && (y==y2);
        intersect3 = (x==x3) && (y==y3);
        intersect4 = (x==x4) && (y==y4);
        
        if ~(intersect1 || intersect2 || intersect3 || intersect4)
            intersectObsTruth = isect;
        end
    end
                
    %k = k + 1;
end

end

