function wallTruth = wallOrNah(robotPose,depthData,hFun,Qcovariance)
%WALLORNAH Is there a wall or nah?
%   Takes info from the stationary robot staring staight at the wall 
%   and makes a decision.
%
%INPUTS:
%   depthData = nx9. Depth data for n times
%   robotPose = x,y,theta of the robot pose.
%   hFun = anonymous measurement function. Should be hDepthMap
%   Qcovariance = sensor noice covariance
%   
%OUTPUTS:
%   wallTruth- true if wall is there, false if it's not


expectedPos = hFun(robotPose)';

el0 = 0;
el_t = el0;
for t = 1:length(depthData(:,1))
    actualDepth = depthData(t,:);
    %[expectedPos' actualDepth']
    
    for i = 1:length(actualDepth(:,1))
        
        probArray = ones(size(expectedPos));
        for j = 1:length(probArray)
            actualDepth_i = actualDepth(j);
            expectedPos_i = expectedPos(j);
            
            probArray(j) = normpdf(actualDepth_i,expectedPos_i,sqrt(Qcovariance));

        end
        probWall = mean(probArray);
        if probWall >= 1
            probWall = 0.9;
        end
        el_t = el_t + log(probWall/(1-probWall)) - el0(i);
    end
    
end

%Turn log odds back to regular prob
prob = exp(el_t) ./ (1 + exp(el_t)); 

%Make a decision
if prob < 0.5
    wallTruth = false;
else
    wallTruth = true;
end

end

