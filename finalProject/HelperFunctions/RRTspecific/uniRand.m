function samples = uniRand(n,mapBounds)
%uniRand: returns a (x,y) coordinate for a map with boundaries mapBounds
%with a uniform random proability distribution
%
%INPUTS:
%   mapBounds: [x1 y1 x2 y2]. (x1,y1) denotes the minimum map (x,y),
%       (x2,y2) denotes the maximum map (x,y). Rectangular map.
%   n = number of points to sample
%OUTPUT:
%   samples: [x,y] coordinate. nx2

    mapBounds = reshape(mapBounds,[4,1]);

    mapBoundsX = [mapBounds(1) mapBounds(3)];
    mapBoundsY = [mapBounds(2) mapBounds(4)];

    lengthX = mapBoundsX(2) - mapBoundsX(1);
    lengthY = mapBoundsY(2) - mapBoundsY(1);

    scaling = [lengthX 0; 0, lengthY]';
    
    samples = zeros(n,2);
    for i = 1:n
        sample = scaling* rand(2,1) + mapBounds(1:2,1);
        sample = sample';
        samples(i,:) = sample;
    end
    %extraSamples = [];
end
