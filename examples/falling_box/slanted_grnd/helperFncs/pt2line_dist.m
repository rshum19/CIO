function [ dist ] = pt2line_dist(p,l)
%PTS2LINE_DIST calculates the distance between a point and a line
%
%   p:      [x,y] point
%   l:   [a,b,c] contact line coefficients  a*x + b*y + c = 0   

x = p(1);
y = p(2);
m = l(1);
c = l(2);

if all(l == 0)
    dist = y;
else
    % Absolute distance
    %dist = abs(m*x + y + c)/sqrt(m^2 + 1);
    
    % Signed distance
    dist = m*x + y + c/sqrt(m^2 + 1);
end

end

