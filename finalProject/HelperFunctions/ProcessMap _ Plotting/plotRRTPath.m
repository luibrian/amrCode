function plotRRTPath(RRT,path,colorsRRT,colorsPath,n)
%PLOTRRTPATH Plots the RRT and the path using the desired color scheme
%
%INPUTS:
%   RRT. Rapidly-exploring random tree. It's a graph containing vertices a
%       edges. 
%   path = nx3. [vertex num, x,y] connect initPoint to goalPoint
%   colorsRRT = cx1 containing possible colors to use to plot RRT
%   colorsPath = cx1 containing possible colors to use to plot path
%   n = which color to use

RRTcolor = colorsRRT(n);
pathColor = colorsPath(n);

nodePos = RRT.Nodes{:,2};
nodeX = nodePos(:,1);
nodeY = nodePos(:,2);
figHdl = plot(RRT,'XData',nodeX,'YData',nodeY);
figHdl.EdgeColor = RRTcolor;

%Plotting the path
highlight(figHdl,path(:,1),'EdgeColor',pathColor,'LineWidth',3);
figHdl.NodeLabel = {};

axis equal;
axis([-3 3 -3 3]);
end

