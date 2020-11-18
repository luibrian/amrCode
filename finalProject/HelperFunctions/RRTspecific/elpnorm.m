function el = elpnorm(point1,point2,p)
%Calculates the Lpnorm between two points
%point1 and point2 are the two points to compare. p is which l_p norm to
%compute
%el is the distance between the two points as calculated by l_p norm

point1 = reshape(point1, [length(point1),1]);
point2 = reshape(point2, [length(point2),1]);

sum = 0;
for i = 1:length(point1)
    sum = sum + abs(point1(i) - point2(i))^p;
end

el = sum^(1/p);
end