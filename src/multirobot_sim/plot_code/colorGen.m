function new_color = colorGen(color)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
   new_color = [0 0 0];
   p = 0.9;
   new_color(1) = 1 - (1 - color(1))*p;
   new_color(2) = 1 - (1 - color(2))*p;
   new_color(3) = 1 - (1 - color(3))*p;
end

