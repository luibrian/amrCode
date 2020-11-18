function depthArray = depthPredict(range)
%DEPTHPREDICT Calculates the expected depth measurements given the expected
%range measurements. This function returns an array of Double, in which
%each entry represents the depth measured from the camera do the obstacle.
%
%   INPUT: range       	K-by-1 vector of ranges (meters)
%
%   OUTPUT: depthArray: array of floats, each element of [depth_array] 
%   represents the depth of the point in meters, from the camera.
%
%   Field of view of depth image is +/- 27 degrees around the sensor-fixed
%   x axis, the first element in the vector is the leftmost point and the
%   last element in the vector is the rightmost point.

npoints = length(range);
angles = (linspace(27,-27,npoints) .* pi/180)';

depthArray = range.*cos(angles);    %depth is always a positive number

end

