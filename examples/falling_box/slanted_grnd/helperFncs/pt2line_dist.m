function [ dist ] = pt2line_dist(p,l)
%PTS2LINE_DIST calculates the distance between a point and a line
%
%   p:      [x,y] point
%   l:   [a,b,c] contact line coefficients  a*x + b*y + c = 0   

x = p(1);
y = p(2);
a = l(1);
b = l(2);
c = l(3);

if all(l == 0)
    dist = y;
else
    dist = abs(a*x + b*y + c)/sqrt(a^2 + b^2);
end

end

