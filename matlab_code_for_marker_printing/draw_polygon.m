function [coord] = draw_polygon( center_loc, rad_circum_circ,sides)
%DRAW_PENTAGON Summary of this function goes here
%   Detailed explanation goes here
coord = zeros(sides+1,2) ;
for i = 1:sides+1
    coord(i,:) =  center_loc + rad_circum_circ*[cos(pi/2 + (i-1)*2*pi/sides), ...
        sin(pi/2 + (i-1)*2*pi/sides)];
end
end

