function [RRT, path, goal] = buildRRT(map,mapBounds,R,initPoint,goalPoint,RRTparam)
%BUILDRRT Generates waypoints using a rapidly-explored random tree
%that connect startLoc and goalLoc for a robot with radius
%   Try to build a map with a bloated R+0.1. If it doesn't work within
%   maxAttempts, decrease the stepsize up until a limit 0.1, then try
%   decreasing the bloat and repeating. 
%   If this still doesn't return a possible RRT, then we done goofed. 
%
%INPUTS:
%   map = mx8 vector of the boundaries
%   mapBounds- boundary of the map
%   R = radius of the robot
%   initPoint = [x,y] of the starting location of robot
%   goalPoint = [x,y] of the desired finish position of the robot
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
%   goal = point the robot is going to 


bloatR = R + 0.1;
path = [];
while bloatR >= R && isempty(path) 
    stepsize = RRTparam.stepsize;
    while stepsize >= 0.1 && isempty(path)
        bloatedMap = createBloatedMap(map,bloatR);

        newMapBounds = mapBounds + [bloatR bloatR -bloatR -bloatR];

        plotMap8(bloatedMap,newMapBounds,1);
        
        RRTparam.sampFunc = @(n) uniRand(n,newMapBounds);
        RRTparam.stepsize = stepsize;
        
        [RRT,path,goal] = buildRRTpoint(bloatedMap,newMapBounds,initPoint,goalPoint,RRTparam);

        if ~isempty(path)
            [RRT, path] = refineRRT(RRT,path,bloatedMap);
        else
            stepsize = stepsize - 0.1;
        end
    end
    bloatR = bloatR - 0.02;
end

end

