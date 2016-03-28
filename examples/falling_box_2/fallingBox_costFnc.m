function [ cost ] = fallingBox_costFnc(t,x,u,params)

nTime = length(t);
nState = size(x,1);
nControl = size(u,1);
xdes = [0;-10];
ydes = -10;
%---- Terminal cost
xend = x(1,round(nTime/2):end);
yend = x(2,end);
terminal_cost = (yend-ydes).^2 ;

%---- Step cost
step_cost = 0;

%---- Total cost
cost = step_cost + terminal_cost;

end

